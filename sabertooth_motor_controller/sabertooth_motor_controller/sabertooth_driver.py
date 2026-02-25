"""Sabertooth motor controller driver via PCA9685 RC PWM.

Translates normalized speed values (-1.0 to +1.0) into PCA9685 duty cycle
values that produce the correct RC PWM pulse widths for Sabertooth operation.

Sabertooth RC mode pulse widths:
  - 1000us = full reverse
  - 1500us = neutral (stopped)
  - 2000us = full forward

This driver works identically with both Sabertooth 2x16 and 2x32 models.
The only difference is current capacity; the PWM interface is the same.

The driver does NOT talk to I2C directly. It delegates to a PWM driver
instance (PCA9685Driver or MockPWMDriver) injected at construction time.
This makes testing straightforward.
"""

import logging
from typing import Tuple

logger = logging.getLogger(__name__)

# PWM constants for Sabertooth RC mode at 50Hz
# These are the foundational calculations for the entire driver stack.
#
# PCA9685 at 50Hz:
#   Period = 1/50 = 20,000 us
#   16-bit resolution = 65535 steps
#
# Conversion: pulse_us -> duty_cycle
#   duty_cycle = 65535 * (pulse_us / 20000)
#
# Key values:
#   1500us (neutral)      -> 65535 * 1500/20000 = 4915
#   2000us (full forward) -> 65535 * 2000/20000 = 6553
#   1000us (full reverse) -> 65535 * 1000/20000 = 3277

DEFAULT_PWM_FREQUENCY = 50
DEFAULT_PULSE_NEUTRAL_US = 1500
DEFAULT_PULSE_MIN_US = 1000
DEFAULT_PULSE_MAX_US = 2000
DEFAULT_DEADBAND_US = 30
DEFAULT_PWM_PERIOD_US = 20000
PWM_RESOLUTION = 65535


class SabertoothDriver:
    """Translates speed commands to Sabertooth-compatible PCA9685 PWM signals.

    This is the core translation layer between the abstract motor speed
    concept (-1.0 to +1.0) and the actual hardware PWM output.

    Example:
        from .pca9685_driver import PCA9685Driver
        pwm = PCA9685Driver(i2c_bus=1)
        driver = SabertoothDriver(pwm, left_channel=0, right_channel=1)
        driver.initialize()
        driver.set_motors(0.5, 0.5)   # Both motors 50% forward
        driver.set_neutral()            # Stop both motors
        driver.shutdown()
    """

    def __init__(
        self,
        pwm_driver,
        left_channel: int = 0,
        right_channel: int = 1,
        pulse_neutral_us: int = DEFAULT_PULSE_NEUTRAL_US,
        pulse_min_us: int = DEFAULT_PULSE_MIN_US,
        pulse_max_us: int = DEFAULT_PULSE_MAX_US,
        deadband_us: int = DEFAULT_DEADBAND_US,
        left_inverted: bool = False,
        right_inverted: bool = False,
        pwm_frequency: int = DEFAULT_PWM_FREQUENCY,
    ):
        """
        Args:
            pwm_driver: PCA9685Driver or MockPWMDriver instance.
                        Must implement set_duty_cycle(channel, duty_cycle).
            left_channel: PCA9685 channel for left motor (0-15).
            right_channel: PCA9685 channel for right motor (0-15).
            pulse_neutral_us: Neutral pulse width in microseconds (default 1500).
            pulse_min_us: Full reverse pulse width (default 1000).
            pulse_max_us: Full forward pulse width (default 2000).
            deadband_us: Speeds producing pulses within this range of neutral
                         snap to exactly neutral. Prevents motor whine at low speeds.
            left_inverted: If True, invert left motor direction. Use this when
                          a motor is wired backwards instead of rewiring.
            right_inverted: If True, invert right motor direction.
            pwm_frequency: PWM frequency in Hz (MUST be 50 for Sabertooth RC mode).
        """
        self._pwm = pwm_driver
        self._left_ch = left_channel
        self._right_ch = right_channel
        self._pulse_neutral = pulse_neutral_us
        self._pulse_min = pulse_min_us
        self._pulse_max = pulse_max_us
        self._deadband = deadband_us
        self._left_inv = left_inverted
        self._right_inv = right_inverted
        self._frequency = pwm_frequency
        self._period_us = 1_000_000 / pwm_frequency  # 20000 at 50Hz
        self._hardware_ready = False

        # Pre-compute neutral duty cycle for fast access
        self._neutral_duty = self.pulse_us_to_duty_cycle(self._pulse_neutral)

    def initialize(self) -> bool:
        """Initialize the underlying PWM driver and set frequency.

        Returns:
            True if hardware is ready, False if running in mock/sim mode.
        """
        self._hardware_ready = self._pwm.initialize()
        if self._hardware_ready or self._pwm.is_initialized():
            self._pwm.set_frequency(self._frequency)
            # Start with both motors at neutral (stopped)
            self.set_neutral()
            logger.info(
                "Sabertooth driver initialized: left=ch%d, right=ch%d, freq=%dHz",
                self._left_ch, self._right_ch, self._frequency
            )
        return self._hardware_ready

    def set_motors(self, left_speed: float, right_speed: float) -> Tuple[int, int]:
        """Set both motor speeds.

        Converts normalized speeds to PWM duty cycles and sends to hardware.
        Applies deadband and motor inversion.

        Args:
            left_speed: Left motor speed, -1.0 (full reverse) to +1.0 (full forward).
            right_speed: Right motor speed, -1.0 (full reverse) to +1.0 (full forward).

        Returns:
            Tuple of (left_duty_cycle, right_duty_cycle) values actually sent.
        """
        # Apply inversion
        left = -left_speed if self._left_inv else left_speed
        right = -right_speed if self._right_inv else right_speed

        # Apply deadband
        left = self._apply_deadband(left)
        right = self._apply_deadband(right)

        # Convert to duty cycles
        left_duty = self._speed_to_duty_cycle(left)
        right_duty = self._speed_to_duty_cycle(right)

        # Send to hardware
        self._pwm.set_duty_cycle(self._left_ch, left_duty)
        self._pwm.set_duty_cycle(self._right_ch, right_duty)

        return (left_duty, right_duty)

    def set_neutral(self) -> None:
        """Set both motors to neutral (stopped).

        Called on timeout, e-stop, shutdown, and initialization.
        Uses the pre-computed neutral duty cycle for speed.
        """
        self._pwm.set_duty_cycle(self._left_ch, self._neutral_duty)
        self._pwm.set_duty_cycle(self._right_ch, self._neutral_duty)

    def shutdown(self) -> None:
        """Set motors to neutral and shutdown the underlying PWM driver.

        IMPORTANT: Always sets neutral BEFORE shutting down the driver.
        If we just deinit the PCA9685, it holds the last PWM value.
        """
        try:
            self.set_neutral()
        except Exception as e:
            logger.error("Error setting neutral during shutdown: %s", e)
        self._pwm.shutdown()
        self._hardware_ready = False
        logger.info("Sabertooth driver shutdown complete")

    def speed_to_pulse_us(self, speed: float) -> float:
        """Convert normalized speed to pulse width in microseconds.

        The mapping is linear:
          speed -1.0 -> pulse_min_us   (1000us = full reverse)
          speed  0.0 -> pulse_neutral  (1500us = stopped)
          speed +1.0 -> pulse_max_us   (2000us = full forward)

        Args:
            speed: Normalized speed, -1.0 to +1.0

        Returns:
            Pulse width in microseconds (clamped to pulse_min..pulse_max)
        """
        speed = max(-1.0, min(1.0, speed))
        half_range = (self._pulse_max - self._pulse_min) / 2.0
        pulse_us = self._pulse_neutral + (speed * half_range)
        return max(self._pulse_min, min(self._pulse_max, pulse_us))

    def pulse_us_to_duty_cycle(self, pulse_us: float) -> int:
        """Convert pulse width (microseconds) to 16-bit PCA9685 duty cycle.

        Formula: duty_cycle = 65535 * (pulse_us / period_us)

        At 50Hz (period = 20000us):
          1000us -> 3277
          1500us -> 4915
          2000us -> 6553

        Args:
            pulse_us: Pulse width in microseconds

        Returns:
            16-bit duty cycle value (0-65535)
        """
        duty = int(PWM_RESOLUTION * (pulse_us / self._period_us))
        return max(0, min(PWM_RESOLUTION, duty))

    def get_neutral_duty_cycle(self) -> int:
        """Return the pre-computed neutral duty cycle value."""
        return self._neutral_duty

    def _speed_to_duty_cycle(self, speed: float) -> int:
        """Convert speed directly to duty cycle (internal convenience)."""
        pulse_us = self.speed_to_pulse_us(speed)
        return self.pulse_us_to_duty_cycle(pulse_us)

    def _apply_deadband(self, speed: float) -> float:
        """Apply deadband - speeds producing pulses within deadband snap to 0.

        This prevents motor whine and jitter at very low speed values.
        The deadband is defined in microseconds around the neutral pulse.

        Args:
            speed: Normalized speed, -1.0 to +1.0

        Returns:
            Speed with deadband applied (small values become 0.0)
        """
        pulse_us = self.speed_to_pulse_us(speed)
        if abs(pulse_us - self._pulse_neutral) < self._deadband:
            return 0.0
        return speed
