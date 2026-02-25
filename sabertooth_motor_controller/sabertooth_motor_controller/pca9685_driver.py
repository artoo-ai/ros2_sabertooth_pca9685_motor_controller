"""PCA9685 PWM driver hardware abstraction layer.

Provides a thread-safe interface to the PCA9685 16-channel PWM controller
connected via I2C. Used to generate RC-style PWM signals (50Hz, 1000-2000us)
for the Sabertooth motor controller.

Hardware connections (Jetson Orin Nano -> PCA9685):
  - Pin 1 (3.3V) -> VCC (logic power - MUST be 3.3V, not 5V)
  - Pin 3 (SDA)  -> SDA
  - Pin 5 (SCL)  -> SCL
  - Pin 9 (GND)  -> GND

If the PCA9685 is not detected, this driver returns False from initialize()
and the node should fall back to MockPWMDriver automatically.

Dependencies:
  - adafruit-circuitpython-pca9685
  - Adafruit-Blinka (provides board/busio modules)
"""

import threading
import logging

logger = logging.getLogger(__name__)


class PCA9685Driver:
    """Thread-safe wrapper around the Adafruit PCA9685 CircuitPython library.

    All I2C operations are serialized through a lock to prevent bus contention
    from the main control loop and shutdown handlers running concurrently.

    Example:
        driver = PCA9685Driver(i2c_bus=1, address=0x40)
        if driver.initialize():
            driver.set_frequency(50)
            driver.set_duty_cycle(0, 4915)  # Channel 0 to neutral (1500us)
        driver.shutdown()
    """

    NUM_CHANNELS = 16
    MAX_DUTY_CYCLE = 65535

    def __init__(self, i2c_bus: int = 1, address: int = 0x40):
        """
        Args:
            i2c_bus: I2C bus number. Jetson Orin Nano typically uses 1 or 7.
                     Run 'i2cdetect -y -r 1' to verify.
            address: PCA9685 I2C address. Default is 0x40.
                     Change if address jumpers are set on the board.
        """
        self._i2c_bus = i2c_bus
        self._address = address
        self._pca = None
        self._i2c = None
        self._initialized = False
        self._lock = threading.Lock()

    def initialize(self) -> bool:
        """Initialize I2C bus and PCA9685 device.

        Attempts to import Adafruit libraries and connect to hardware.
        If any step fails (wrong platform, hardware not connected, etc.),
        logs the error and returns False.

        Returns:
            True if hardware found and initialized successfully.
            False if hardware not available (caller should use MockPWMDriver).
        """
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685

            # Try to open I2C bus
            # On Jetson Orin Nano, board.SCL/SDA map to the correct bus
            # If that fails, try board.SCL_1/SDA_1 (bus 1 alternate)
            try:
                self._i2c = busio.I2C(board.SCL, board.SDA)
            except Exception:
                try:
                    self._i2c = busio.I2C(board.SCL_1, board.SDA_1)
                except Exception as e:
                    logger.error("Failed to open I2C bus: %s", e)
                    return False

            # Try to connect to PCA9685
            self._pca = PCA9685(self._i2c, address=self._address)
            self._initialized = True
            logger.info(
                "PCA9685 initialized on I2C bus %d at address 0x%02X",
                self._i2c_bus, self._address
            )
            return True

        except ImportError as e:
            logger.warning(
                "Adafruit libraries not available (expected on non-Jetson): %s", e
            )
            return False
        except ValueError as e:
            logger.error("PCA9685 not found at address 0x%02X: %s", self._address, e)
            return False
        except Exception as e:
            logger.error("Unexpected error initializing PCA9685: %s", e)
            return False

    def set_frequency(self, freq_hz: int) -> None:
        """Set PWM frequency for all channels.

        For Sabertooth RC mode, this MUST be 50Hz (standard RC servo frequency).
        The PCA9685 supports 24Hz to 1526Hz.

        Args:
            freq_hz: Frequency in Hz (use 50 for Sabertooth RC mode)

        Raises:
            RuntimeError: If not initialized
        """
        if not self._initialized:
            raise RuntimeError("PCA9685 not initialized. Call initialize() first.")
        with self._lock:
            self._pca.frequency = freq_hz
            logger.info("PCA9685 frequency set to %d Hz", freq_hz)

    def set_duty_cycle(self, channel: int, duty_cycle: int) -> None:
        """Set 16-bit duty cycle on a specific channel. Thread-safe.

        The PCA9685 Adafruit library uses 16-bit duty cycle values (0-65535).
        For Sabertooth at 50Hz:
          - Neutral (1500us): duty_cycle = 4915
          - Full forward (2000us): duty_cycle = 6553
          - Full reverse (1000us): duty_cycle = 3277

        Args:
            channel: PCA9685 channel number (0-15)
            duty_cycle: 16-bit duty cycle value (0-65535)

        Raises:
            ValueError: If channel or duty_cycle out of valid range
            RuntimeError: If not initialized
        """
        if not self._initialized:
            raise RuntimeError("PCA9685 not initialized. Call initialize() first.")
        if not 0 <= channel < self.NUM_CHANNELS:
            raise ValueError(f"Channel {channel} out of range (0-{self.NUM_CHANNELS - 1})")
        if not 0 <= duty_cycle <= self.MAX_DUTY_CYCLE:
            raise ValueError(f"Duty cycle {duty_cycle} out of range (0-{self.MAX_DUTY_CYCLE})")

        with self._lock:
            try:
                self._pca.channels[channel].duty_cycle = duty_cycle
            except Exception as e:
                logger.error("I2C write failed on channel %d: %s", channel, e)
                raise

    def set_all_neutral(self, neutral_duty: int = 4915) -> None:
        """Set all 16 channels to neutral duty cycle.

        Used for emergency stop and shutdown. Sets every channel,
        not just the motor channels, as a safety measure.

        Args:
            neutral_duty: Duty cycle for neutral position.
                          Default 4915 = 1500us at 50Hz = Sabertooth neutral.
        """
        for ch in range(self.NUM_CHANNELS):
            try:
                self.set_duty_cycle(ch, neutral_duty)
            except Exception as e:
                logger.error("Failed to set channel %d to neutral: %s", ch, e)

    def shutdown(self) -> None:
        """Set all channels to neutral and deinitialize hardware.

        Safe to call multiple times. Always attempts to set neutral
        before closing the I2C connection, even if errors occur.

        IMPORTANT: This is called during node shutdown. Motors MUST
        return to neutral (1500us = stopped) before we release the
        I2C bus, otherwise the PCA9685 holds the last PWM value.
        """
        if self._initialized:
            try:
                self.set_all_neutral()
            except Exception as e:
                logger.error("Error setting neutral during shutdown: %s", e)

            try:
                with self._lock:
                    if self._pca is not None:
                        self._pca.deinit()
                        self._pca = None
                    if self._i2c is not None:
                        self._i2c.deinit()
                        self._i2c = None
            except Exception as e:
                logger.error("Error during PCA9685 deinit: %s", e)

            self._initialized = False
            logger.info("PCA9685 shutdown complete")

    def is_initialized(self) -> bool:
        """Return True if hardware is initialized and ready."""
        return self._initialized
