"""Mock hardware drivers for development and testing without real hardware.

Provides drop-in replacements for PCA9685Driver and RCInput that work
on any platform (MacBook, CI, etc). Used automatically when:
  - simulation_mode parameter is True
  - PCA9685 hardware is not detected on I2C
  - Running on a non-Jetson platform

MockPWMDriver records all duty cycle changes for test verification.
MockRCInput allows injecting simulated RC pulses for testing priority logic.
"""

import time
import threading
from typing import List, Tuple, Optional, NamedTuple


class MockPWMDriver:
    """Mock PCA9685 driver that records all operations in memory.

    Implements the same interface as PCA9685Driver. All duty cycle values
    are stored and can be queried for verification in tests.

    Example:
        driver = MockPWMDriver()
        driver.initialize()
        driver.set_duty_cycle(0, 4915)      # channel 0 to neutral
        assert driver.get_duty_cycle(0) == 4915
    """

    NUM_CHANNELS = 16
    MAX_DUTY_CYCLE = 65535

    def __init__(self, i2c_bus: int = 1, address: int = 0x40):
        self._i2c_bus = i2c_bus
        self._address = address
        self._initialized = False
        self._frequency = 0
        self._channels = [0] * self.NUM_CHANNELS
        self._history: List[Tuple[float, int, int]] = []  # (timestamp, channel, duty_cycle)
        self._lock = threading.Lock()

    def initialize(self) -> bool:
        """Always succeeds in mock mode."""
        self._initialized = True
        return True

    def set_frequency(self, freq_hz: int) -> None:
        """Record the frequency setting."""
        if not self._initialized:
            raise RuntimeError("MockPWMDriver not initialized")
        self._frequency = freq_hz

    def set_duty_cycle(self, channel: int, duty_cycle: int) -> None:
        """Record duty cycle for a channel. Thread-safe.

        Args:
            channel: PCA9685 channel (0-15)
            duty_cycle: 16-bit value (0-65535)

        Raises:
            ValueError: If channel or duty_cycle out of range
            RuntimeError: If not initialized
        """
        if not self._initialized:
            raise RuntimeError("MockPWMDriver not initialized")
        if not 0 <= channel < self.NUM_CHANNELS:
            raise ValueError(f"Channel {channel} out of range (0-{self.NUM_CHANNELS - 1})")
        if not 0 <= duty_cycle <= self.MAX_DUTY_CYCLE:
            raise ValueError(f"Duty cycle {duty_cycle} out of range (0-{self.MAX_DUTY_CYCLE})")

        with self._lock:
            self._channels[channel] = duty_cycle
            self._history.append((time.monotonic(), channel, duty_cycle))

    def set_all_neutral(self, neutral_duty: int = 4915) -> None:
        """Set all channels to neutral duty cycle."""
        for ch in range(self.NUM_CHANNELS):
            self.set_duty_cycle(ch, neutral_duty)

    def shutdown(self) -> None:
        """Set all channels to 0 and mark as not initialized."""
        if self._initialized:
            self.set_all_neutral()
            self._initialized = False

    def is_initialized(self) -> bool:
        return self._initialized

    # --- Test helpers ---

    def get_duty_cycle(self, channel: int) -> int:
        """Get the last duty cycle value set on a channel."""
        with self._lock:
            return self._channels[channel]

    def get_history(self, channel: Optional[int] = None) -> List[Tuple[float, int, int]]:
        """Get timestamped history of duty cycle changes.

        Args:
            channel: If specified, filter to this channel only.

        Returns:
            List of (timestamp, channel, duty_cycle) tuples.
        """
        with self._lock:
            if channel is not None:
                return [(t, ch, dc) for t, ch, dc in self._history if ch == channel]
            return list(self._history)

    def reset_history(self) -> None:
        """Clear all recorded history."""
        with self._lock:
            self._history.clear()


class MockRCChannel(NamedTuple):
    """Snapshot of a single RC channel state."""
    pulse_us: float
    valid: bool
    age_sec: float


class MockRCInputState(NamedTuple):
    """Complete RC input state snapshot."""
    ch1: MockRCChannel
    ch2: MockRCChannel
    ch3: MockRCChannel
    connected: bool
    estop_active: bool


class MockRCInput:
    """Mock RC input for testing without a real RC receiver.

    Allows tests to inject simulated RC pulses and control
    the mock RC state programmatically.

    Example:
        rc = MockRCInput()
        rc.start()
        rc.simulate_pulse(1, 1800.0)   # Ch1 forward
        rc.simulate_pulse(2, 1500.0)   # Ch2 neutral
        speeds = rc.get_motor_speeds()  # Returns motor speeds from RC
    """

    def __init__(self, **kwargs):
        """Accept same kwargs as RCInput, store for reference."""
        self._config = kwargs
        self._pulse_min_us = kwargs.get('pulse_min_us', 1000.0)
        self._pulse_max_us = kwargs.get('pulse_max_us', 2000.0)
        self._pulse_neutral_us = kwargs.get('pulse_neutral_us', 1500.0)
        self._deadband_us = kwargs.get('deadband_us', 50.0)
        self._estop_threshold_us = kwargs.get('estop_threshold_us', 1200.0)
        self._rc_mixing_mode = kwargs.get('rc_mixing_mode', 'tank')
        self._started = False
        self._connected = False
        self._estop_active = False
        self._lock = threading.Lock()

        # Simulated channel pulse widths (0 = no signal)
        self._ch_pulses = {1: 0.0, 2: 0.0, 3: 0.0}
        self._ch_times = {1: 0.0, 2: 0.0, 3: 0.0}

    def start(self) -> bool:
        """Always succeeds in mock mode."""
        self._started = True
        return True

    def stop(self) -> None:
        """No-op cleanup."""
        self._started = False

    def get_state(self) -> MockRCInputState:
        """Return current simulated state."""
        with self._lock:
            now = time.monotonic()

            def _make_channel(ch_num: int) -> MockRCChannel:
                pulse = self._ch_pulses[ch_num]
                ch_time = self._ch_times[ch_num]
                age = now - ch_time if ch_time > 0 else float('inf')
                valid = (self._pulse_min_us <= pulse <= self._pulse_max_us) and age < 0.5
                return MockRCChannel(pulse_us=pulse, valid=valid, age_sec=age)

            ch1 = _make_channel(1)
            ch2 = _make_channel(2)
            ch3 = _make_channel(3)

            return MockRCInputState(
                ch1=ch1,
                ch2=ch2,
                ch3=ch3,
                connected=self._connected,
                estop_active=self._estop_active,
            )

    def get_motor_speeds(self) -> Optional[Tuple[float, float]]:
        """Convert current RC input to motor speeds if RC is active.

        Returns:
            (left_speed, right_speed) if connected, None otherwise.
        """
        with self._lock:
            if not self._connected:
                return None

            ch1_speed = self._pulse_to_speed(self._ch_pulses[1])
            ch2_speed = self._pulse_to_speed(self._ch_pulses[2])

            if self._rc_mixing_mode == 'tank':
                # Tank mode: ch1 = left motor, ch2 = right motor
                return (ch1_speed, ch2_speed)
            else:
                # Arcade mode: ch1 = steering, ch2 = throttle
                left = ch2_speed + ch1_speed
                right = ch2_speed - ch1_speed
                # Normalize if either exceeds 1.0
                max_val = max(abs(left), abs(right), 1.0)
                return (left / max_val, right / max_val)

    def _pulse_to_speed(self, pulse_us: float) -> float:
        """Convert pulse width to normalized speed with deadband."""
        if pulse_us < self._pulse_min_us or pulse_us > self._pulse_max_us:
            return 0.0

        # Center around neutral
        offset = pulse_us - self._pulse_neutral_us

        # Apply deadband
        if abs(offset) < self._deadband_us:
            return 0.0

        # Normalize to -1.0..+1.0
        half_range = (self._pulse_max_us - self._pulse_min_us) / 2.0
        speed = offset / half_range
        return max(-1.0, min(1.0, speed))

    # --- Test helpers ---

    def simulate_pulse(self, channel: int, pulse_us: float) -> None:
        """Inject a simulated RC pulse.

        Args:
            channel: RC channel number (1, 2, or 3)
            pulse_us: Pulse width in microseconds
        """
        with self._lock:
            self._ch_pulses[channel] = pulse_us
            self._ch_times[channel] = time.monotonic()
            # Auto-detect connected state: at least ch1 and ch2 have valid pulses
            self._connected = (
                self._pulse_min_us <= self._ch_pulses[1] <= self._pulse_max_us
                and self._pulse_min_us <= self._ch_pulses[2] <= self._pulse_max_us
            )
            # Check e-stop
            if self._ch_pulses[3] > 0:
                self._estop_active = self._ch_pulses[3] < self._estop_threshold_us

    def simulate_connected(self, connected: bool) -> None:
        """Directly set the connected state."""
        with self._lock:
            self._connected = connected

    def simulate_signal_lost(self) -> None:
        """Simulate loss of all RC signals."""
        with self._lock:
            self._connected = False
            self._ch_pulses = {1: 0.0, 2: 0.0, 3: 0.0}
            self._ch_times = {1: 0.0, 2: 0.0, 3: 0.0}

    def simulate_estop(self, active: bool) -> None:
        """Directly set e-stop state."""
        with self._lock:
            self._estop_active = active
            if active:
                self._ch_pulses[3] = self._estop_threshold_us - 100
            else:
                self._ch_pulses[3] = self._pulse_neutral_us
            self._ch_times[3] = time.monotonic()
