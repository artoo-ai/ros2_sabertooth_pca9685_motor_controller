"""RC receiver PWM input reader using GPIO edge detection.

Reads standard RC PWM signals (1000-2000us pulses at ~50Hz) from
FlySky-type receivers connected to Jetson Orin Nano GPIO pins.

HOW IT WORKS:
  Since Jetson.GPIO does NOT support hardware PWM input capture,
  this module uses GPIO.BOTH edge detection with high-resolution
  timing to measure pulse widths:

  RC Receiver Output (per channel, repeating at 50Hz):
      ┌──────┐                              ┌──────┐
      │      │                              │      │
  ────┘      └──────────────────────────────┘      └────
      ^      ^                              ^      ^
    RISE   FALL                           RISE   FALL
      |      |
      +------+  = pulse width (1000-2000 us)
                  measured via time.monotonic_ns()

  Each edge triggers a callback in a separate thread.
  RISING edge: record timestamp
  FALLING edge: compute pulse_width = now - rising_timestamp

ACCURACY:
  - time.monotonic_ns() provides nanosecond-resolution timestamps
  - Python GIL and OS scheduling can introduce 50-200us jitter
  - For RC control this is acceptable: we need ~50us resolution
    out of a 1000us range (5% accuracy)
  - We use a wide acceptance window (800-2200us) for jitter tolerance
  - The Sabertooth's own input smoothing helps absorb jitter
  - Hardware e-stop exists as the real failsafe

WIRING:
  - RC receiver signal wires -> Jetson GPIO header pins
  - RC receiver GND -> Jetson GND (Pin 6) - MUST share common ground
  - RC receiver VCC -> 5V BEC (NOT from Jetson 3.3V pin)
  - RC signal levels (3.3V-5V) are safe for Jetson GPIO inputs

GPIO PIN ASSIGNMENT (configurable via ROS params):
  - Pin 18: RC Channel 1 (left/right or steering)
  - Pin 22: RC Channel 2 (forward/back or throttle)
  - Pin 13: RC Channel 3 (e-stop switch on transmitter)
"""

import time
import threading
import logging
from typing import Optional, Tuple, NamedTuple

logger = logging.getLogger(__name__)


class RCChannel(NamedTuple):
    """Snapshot of a single RC channel state."""
    pulse_us: float    # Last measured pulse width (microseconds)
    valid: bool        # True if pulse is within valid range and recent
    age_sec: float     # Seconds since last valid pulse


class RCInputState(NamedTuple):
    """Complete RC input state snapshot (thread-safe copy)."""
    ch1: RCChannel
    ch2: RCChannel
    ch3: RCChannel        # E-stop channel
    connected: bool       # True if ch1 AND ch2 receiving valid signals
    estop_active: bool    # True if ch3 indicates e-stop engaged


class _ChannelData:
    """Internal mutable state for one RC channel."""

    def __init__(self):
        self.rise_time_ns: int = 0
        self.pulse_us: float = 0.0
        self.last_valid_time: float = 0.0
        self.lock = threading.Lock()


class RCInput:
    """Reads RC receiver PWM signals via Jetson GPIO edge detection.

    Usage:
        rc = RCInput(ch1_pin=18, ch2_pin=22, ch3_pin=13)
        if rc.start():
            state = rc.get_state()
            if state.connected:
                speeds = rc.get_motor_speeds()
        rc.stop()
    """

    # Acceptance window wider than RC spec to handle timing jitter
    _PULSE_ACCEPT_MIN_US = 800.0
    _PULSE_ACCEPT_MAX_US = 2200.0

    def __init__(
        self,
        ch1_pin: int = 18,
        ch2_pin: int = 22,
        ch3_pin: int = 13,
        pin_numbering: str = "BOARD",
        pulse_min_us: float = 1000.0,
        pulse_max_us: float = 2000.0,
        pulse_neutral_us: float = 1500.0,
        deadband_us: float = 50.0,
        signal_timeout_ms: float = 500.0,
        estop_threshold_us: float = 1200.0,
        rc_mixing_mode: str = "tank",
    ):
        """
        Args:
            ch1_pin: GPIO header pin for channel 1 (BOARD numbering).
            ch2_pin: GPIO header pin for channel 2.
            ch3_pin: GPIO header pin for channel 3 (e-stop switch).
            pin_numbering: "BOARD" (physical pin number) or "BCM" (GPIO number).
            pulse_min_us: Minimum valid pulse width for speed mapping.
            pulse_max_us: Maximum valid pulse width for speed mapping.
            pulse_neutral_us: Center/neutral pulse width.
            deadband_us: Deadband around neutral (reduces drift and jitter).
            signal_timeout_ms: Time without pulses before declaring signal lost.
                               At 50Hz, 500ms = 25 missed frames.
            estop_threshold_us: Ch3 pulse below this value = e-stop active.
                                Typically a toggle switch on the transmitter.
            rc_mixing_mode: "tank" (ch1=left, ch2=right) or
                           "arcade" (ch1=steering, ch2=throttle).
        """
        self._pins = {1: ch1_pin, 2: ch2_pin, 3: ch3_pin}
        self._pin_numbering = pin_numbering
        self._pulse_min = pulse_min_us
        self._pulse_max = pulse_max_us
        self._pulse_neutral = pulse_neutral_us
        self._deadband = deadband_us
        self._signal_timeout_sec = signal_timeout_ms / 1000.0
        self._estop_threshold = estop_threshold_us
        self._mixing_mode = rc_mixing_mode

        # Channel data (populated in start())
        self._channels: dict = {}
        self._pin_to_channel: dict = {}
        self._gpio_available = False
        self._started = False

    def start(self) -> bool:
        """Begin listening for RC signals on configured GPIO pins.

        Sets up GPIO edge detection callbacks for all three channels.
        If GPIO library is not available (e.g., running on Mac), returns False.

        Returns:
            True if GPIO setup succeeded, False if unavailable.
        """
        try:
            import Jetson.GPIO as GPIO

            # Set pin numbering mode
            if self._pin_numbering == "BOARD":
                GPIO.setmode(GPIO.BOARD)
            else:
                GPIO.setmode(GPIO.BCM)

            GPIO.setwarnings(False)

            # Set up each channel
            for ch_num, pin in self._pins.items():
                self._channels[ch_num] = _ChannelData()
                self._pin_to_channel[pin] = ch_num

                GPIO.setup(pin, GPIO.IN)
                GPIO.add_event_detect(
                    pin,
                    GPIO.BOTH,
                    callback=self._edge_callback,
                    bouncetime=0,  # No debounce - we need precise timing
                )

            self._gpio_available = True
            self._started = True
            logger.info(
                "RC input started: ch1=pin%d, ch2=pin%d, ch3=pin%d, mode=%s",
                self._pins[1], self._pins[2], self._pins[3], self._mixing_mode,
            )
            return True

        except ImportError:
            logger.warning(
                "Jetson.GPIO not available (expected on non-Jetson platforms). "
                "RC input disabled. Use MockRCInput for testing."
            )
            return False
        except Exception as e:
            logger.error("Failed to set up GPIO for RC input: %s", e)
            return False

    def stop(self) -> None:
        """Stop listening and clean up GPIO resources."""
        if self._gpio_available:
            try:
                import Jetson.GPIO as GPIO
                for pin in self._pins.values():
                    GPIO.remove_event_detect(pin)
                GPIO.cleanup(list(self._pins.values()))
            except Exception as e:
                logger.error("Error cleaning up GPIO: %s", e)
        self._started = False
        logger.info("RC input stopped")

    def get_state(self) -> RCInputState:
        """Get current RC input state as a thread-safe snapshot.

        Returns:
            RCInputState with current pulse widths, validity, and status flags.
        """
        now = time.monotonic()

        def _read_channel(ch_num: int) -> RCChannel:
            if ch_num not in self._channels:
                return RCChannel(pulse_us=0.0, valid=False, age_sec=float('inf'))

            ch = self._channels[ch_num]
            with ch.lock:
                pulse = ch.pulse_us
                last_time = ch.last_valid_time

            age = now - last_time if last_time > 0 else float('inf')
            valid = (
                self._pulse_min <= pulse <= self._pulse_max
                and age < self._signal_timeout_sec
            )
            return RCChannel(pulse_us=pulse, valid=valid, age_sec=age)

        ch1 = _read_channel(1)
        ch2 = _read_channel(2)
        ch3 = _read_channel(3)

        # Connected = both drive channels have valid signals
        connected = ch1.valid and ch2.valid

        # E-stop active = ch3 pulse below threshold (switch in "stop" position)
        # If ch3 has no signal, we do NOT trigger e-stop (allows use without ch3)
        estop_active = ch3.valid and ch3.pulse_us < self._estop_threshold

        return RCInputState(
            ch1=ch1,
            ch2=ch2,
            ch3=ch3,
            connected=connected,
            estop_active=estop_active,
        )

    def get_motor_speeds(self) -> Optional[Tuple[float, float]]:
        """Convert current RC input to motor speeds if RC is active.

        Applies deadband and the configured mixing mode (tank or arcade).

        Returns:
            (left_speed, right_speed) if RC connected, None if not connected.
        """
        state = self.get_state()
        if not state.connected:
            return None

        ch1_speed = self._pulse_to_speed(state.ch1.pulse_us)
        ch2_speed = self._pulse_to_speed(state.ch2.pulse_us)

        if self._mixing_mode == "tank":
            # Tank mode: ch1 = left motor, ch2 = right motor
            return (ch1_speed, ch2_speed)
        else:
            # Arcade mode: ch1 = steering (left/right), ch2 = throttle (fwd/back)
            throttle = ch2_speed
            steering = ch1_speed
            left = throttle + steering
            right = throttle - steering
            # Normalize if either exceeds 1.0, preserving ratio
            max_val = max(abs(left), abs(right), 1.0)
            return (left / max_val, right / max_val)

    def _edge_callback(self, channel: int) -> None:
        """GPIO edge detection callback (called in a separate thread by GPIO library).

        On RISING edge: record the timestamp (start of pulse)
        On FALLING edge: compute pulse width = now - rising_timestamp

        Uses time.monotonic_ns() for nanosecond precision.

        Args:
            channel: The GPIO pin number that triggered the callback.
        """
        try:
            import Jetson.GPIO as GPIO
        except ImportError:
            return

        now_ns = time.monotonic_ns()
        pin_state = GPIO.input(channel)

        ch_num = self._pin_to_channel.get(channel)
        if ch_num is None:
            return

        ch_data = self._channels.get(ch_num)
        if ch_data is None:
            return

        if pin_state == GPIO.HIGH:
            # Rising edge - start timing
            ch_data.rise_time_ns = now_ns
        else:
            # Falling edge - compute pulse width
            if ch_data.rise_time_ns > 0:
                pulse_ns = now_ns - ch_data.rise_time_ns
                pulse_us = pulse_ns / 1000.0

                # Validate: reject noise and glitches
                if self._PULSE_ACCEPT_MIN_US <= pulse_us <= self._PULSE_ACCEPT_MAX_US:
                    with ch_data.lock:
                        ch_data.pulse_us = pulse_us
                        ch_data.last_valid_time = time.monotonic()

    def _pulse_to_speed(self, pulse_us: float) -> float:
        """Convert RC pulse width to normalized speed with deadband.

        Mapping:
          pulse_min_us (1000)    -> -1.0 (full reverse)
          pulse_neutral_us (1500) ->  0.0 (stopped)
          pulse_max_us (2000)    -> +1.0 (full forward)

        Args:
            pulse_us: Pulse width in microseconds.

        Returns:
            Speed from -1.0 to +1.0, with deadband applied around neutral.
        """
        if pulse_us < self._pulse_min or pulse_us > self._pulse_max:
            return 0.0

        # Center around neutral
        offset = pulse_us - self._pulse_neutral

        # Apply deadband
        if abs(offset) < self._deadband:
            return 0.0

        # Normalize to -1.0..+1.0
        half_range = (self._pulse_max - self._pulse_min) / 2.0
        if half_range == 0:
            return 0.0

        speed = offset / half_range
        return max(-1.0, min(1.0, speed))
