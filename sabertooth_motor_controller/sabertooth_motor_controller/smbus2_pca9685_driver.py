"""PCA9685 PWM driver using smbus2 for direct I2C register access.

Bypasses Adafruit Blinka/CircuitPython which has compatibility issues
with Jetson Orin Nano ("Could not determine Jetson model" error).
Instead, talks directly to the PCA9685 via Linux I2C device files
(/dev/i2c-N) using the smbus2 library.

This is the preferred driver on Jetson platforms. Falls back to the
Adafruit-based PCA9685Driver if smbus2 is not available.

PCA9685 Register Map (relevant registers):
  MODE1    (0x00): Mode register 1 - sleep, auto-inc, restart, allcall
  MODE2    (0x01): Mode register 2 - output driver config
  LED0_ON  (0x06): Channel 0 ON count low byte (4 bytes per channel)
  ...
  LED15_ON (0x42): Channel 15 ON count low byte
  ALL_ON   (0xFA): All channels ON count low byte
  ALL_OFF  (0xFC): All channels OFF count low byte
  PRESCALE (0xFE): Frequency prescaler (writable only in sleep mode)

16-bit to 12-bit duty cycle conversion:
  The rest of the driver stack uses 16-bit duty cycle values (0-65535)
  for compatibility with the Adafruit PCA9685 library convention.
  The PCA9685 hardware uses 12-bit counters (0-4095). Conversion:
    12bit_value = (16bit_value + 1) >> 4

  At 50Hz with 12-bit resolution (4096 counts per 20ms period):
    Neutral (1500us): 16-bit=4915  -> 12-bit=307  -> 307/4096*20000 = 1499us
    Full fwd (2000us): 16-bit=6553 -> 12-bit=409  -> 409/4096*20000 = 1997us
    Full rev (1000us): 16-bit=3277 -> 12-bit=204  -> 204/4096*20000 =  996us

I2C Bus Auto-Detection:
  On Jetson Orin Nano, the 40-pin header I2C often maps to /dev/i2c-7
  rather than /dev/i2c-1. This driver tries the configured bus first,
  then scans other common buses. Devices showing as 'UU' in i2cdetect
  are kernel-claimed and skipped (to avoid conflicts with kernel drivers).

Dependencies:
  - smbus2 (pip install smbus2, or apt install python3-smbus2)
"""

import os
import time
import threading
import logging

logger = logging.getLogger(__name__)


class SMBus2PCA9685Driver:
    """Thread-safe PCA9685 driver using smbus2 for direct I2C register access.

    Drop-in replacement for PCA9685Driver. Implements the same interface
    (initialize, set_frequency, set_duty_cycle, set_all_neutral, shutdown,
    is_initialized) but uses smbus2 instead of Adafruit CircuitPython.

    Example:
        driver = SMBus2PCA9685Driver(i2c_bus=7, address=0x40)
        if driver.initialize():
            driver.set_frequency(50)
            driver.set_duty_cycle(0, 4915)  # Channel 0 to neutral (1500us)
        driver.shutdown()
    """

    # =========================================================================
    # PCA9685 Register Addresses
    # =========================================================================

    _REG_MODE1 = 0x00
    _REG_MODE2 = 0x01
    _REG_LED0_ON_L = 0x06       # First channel register (4 bytes per channel)
    _REG_ALL_LED_ON_L = 0xFA    # Bulk write: all channels ON low byte
    _REG_ALL_LED_OFF_L = 0xFC   # Bulk write: all channels OFF low byte
    _REG_PRESCALE = 0xFE        # Frequency prescaler

    # =========================================================================
    # MODE1 Register Bits
    # =========================================================================

    _MODE1_RESTART = 0x80   # Bit 7: Restart enabled
    _MODE1_EXTCLK = 0x40    # Bit 6: External clock (not used)
    _MODE1_AI = 0x20        # Bit 5: Auto-Increment register pointer
    _MODE1_SLEEP = 0x10     # Bit 4: Sleep mode (oscillator off)
    _MODE1_ALLCALL = 0x01   # Bit 0: Respond to all-call address (0x70)

    # =========================================================================
    # MODE2 Register Bits
    # =========================================================================

    _MODE2_OUTDRV = 0x04    # Bit 2: Totem-pole output (vs open-drain)

    # =========================================================================
    # Hardware Constants
    # =========================================================================

    _OSC_CLOCK_HZ = 25_000_000   # PCA9685 internal oscillator frequency
    _PCA9685_ALLCALL_ADDR = 0x70  # Default all-call address

    NUM_CHANNELS = 16
    MAX_DUTY_CYCLE = 65535

    # Common Jetson Orin Nano I2C buses (40-pin header)
    _COMMON_BUSES = [7, 8, 1, 0]

    def __init__(self, i2c_bus: int = 1, address: int = 0x40):
        """
        Args:
            i2c_bus: Preferred I2C bus number. If the PCA9685 is not found
                     on this bus, other common buses will be scanned.
                     Jetson Orin Nano typically uses bus 7 for the 40-pin header.
                     Run 'i2cdetect -y -r 7' to verify.
            address: PCA9685 I2C address. Default is 0x40.
                     Change if address jumpers are set on the board.
        """
        self._configured_bus = i2c_bus
        self._address = address
        self._bus = None
        self._actual_bus_num = None
        self._initialized = False
        self._lock = threading.Lock()

    def initialize(self) -> bool:
        """Initialize I2C bus and PCA9685 device.

        Attempts to find the PCA9685 on the configured I2C bus first,
        then scans other common Jetson buses. Once found, resets the
        PCA9685 to a known good state.

        The PCA9685 is identified by successfully reading its MODE1
        register and optionally confirming the all-call address (0x70)
        responds on the same bus.

        Returns:
            True if hardware found and initialized successfully.
            False if smbus2 not available or PCA9685 not found.
        """
        # Idempotent: if already initialized, return True
        if self._initialized:
            return True

        try:
            from smbus2 import SMBus
        except ImportError:
            logger.warning(
                "smbus2 not available. Install with: "
                "pip install smbus2  (or: apt install python3-smbus2)"
            )
            return False

        # Build ordered list of buses to try: configured bus first, then others
        buses_to_try = [self._configured_bus]
        for bus_num in self._COMMON_BUSES:
            if bus_num not in buses_to_try:
                buses_to_try.append(bus_num)

        # Scan buses for PCA9685
        for bus_num in buses_to_try:
            dev_path = f"/dev/i2c-{bus_num}"
            if not os.path.exists(dev_path):
                logger.debug("I2C bus %d not available (%s does not exist)", bus_num, dev_path)
                continue

            try:
                bus = SMBus(bus_num)

                # Try reading MODE1 register to verify a device responds
                mode1 = bus.read_byte_data(self._address, self._REG_MODE1)

                # Additional check: see if the PCA9685 all-call address (0x70)
                # also responds on this bus. This helps distinguish a real PCA9685
                # from other devices that happen to be at 0x40.
                allcall_found = False
                try:
                    bus.read_byte_data(self._PCA9685_ALLCALL_ADDR, self._REG_MODE1)
                    allcall_found = True
                except Exception:
                    pass

                logger.info(
                    "PCA9685 found on I2C bus %d at 0x%02X "
                    "(MODE1=0x%02X, allcall_0x70=%s)",
                    bus_num, self._address, mode1,
                    "yes" if allcall_found else "no"
                )

                self._bus = bus
                self._actual_bus_num = bus_num
                break

            except Exception as e:
                logger.debug("PCA9685 not found on bus %d: %s", bus_num, e)
                try:
                    bus.close()
                except Exception:
                    pass
                continue

        if self._bus is None:
            logger.error(
                "PCA9685 not found at 0x%02X on any I2C bus. "
                "Tried buses: %s. Run 'i2cdetect -y -r <bus>' to check.",
                self._address, buses_to_try
            )
            return False

        # Reset PCA9685 to known good state
        try:
            self._reset_device()
            self._initialized = True

            if self._actual_bus_num != self._configured_bus:
                logger.warning(
                    "PCA9685 found on bus %d (configured bus was %d). "
                    "Consider setting hardware.i2c_bus: %d in your config.",
                    self._actual_bus_num, self._configured_bus, self._actual_bus_num
                )
            return True

        except Exception as e:
            logger.error("Failed to configure PCA9685 on bus %d: %s", self._actual_bus_num, e)
            self._cleanup_bus()
            return False

    def _reset_device(self) -> None:
        """Reset PCA9685 to a known good state.

        Sequence:
          1. Sleep (stop oscillator, required before changing prescale)
          2. Set MODE2 for totem-pole outputs
          3. Wake up with auto-increment enabled
          4. Wait for oscillator to stabilize (500us minimum per datasheet)
        """
        # Put device to sleep (oscillator off)
        self._bus.write_byte_data(self._address, self._REG_MODE1, self._MODE1_SLEEP)
        time.sleep(0.005)  # 5ms for oscillator to fully stop

        # Configure MODE2: totem-pole output drivers
        self._bus.write_byte_data(self._address, self._REG_MODE2, self._MODE2_OUTDRV)

        # Wake up with auto-increment and all-call enabled
        wake_mode = self._MODE1_AI | self._MODE1_ALLCALL
        self._bus.write_byte_data(self._address, self._REG_MODE1, wake_mode)
        time.sleep(0.005)  # Wait for oscillator to start (500us min per datasheet)

        logger.debug("PCA9685 reset complete on bus %d", self._actual_bus_num)

    def set_frequency(self, freq_hz: int) -> None:
        """Set PWM frequency for all channels.

        For Sabertooth RC mode, this MUST be 50Hz (standard RC servo frequency).
        The PCA9685 supports approximately 24Hz to 1526Hz.

        PCA9685 prescale formula:
          prescale = round(25MHz / (4096 * desired_freq)) - 1

        For 50Hz: prescale = round(25000000 / 204800) - 1 = 122 - 1 = 121
        Actual frequency with prescale=121: 25MHz / (4096 * 122) = 50.03Hz

        IMPORTANT: The prescaler register can only be written while the
        device is in sleep mode. This method handles the sleep/wake cycle.

        Args:
            freq_hz: Frequency in Hz (use 50 for Sabertooth RC mode).

        Raises:
            RuntimeError: If not initialized.
        """
        if not self._initialized:
            raise RuntimeError("PCA9685 not initialized. Call initialize() first.")

        # Calculate prescale value (clamped to valid 8-bit range 3-255)
        prescale = round(self._OSC_CLOCK_HZ / (4096.0 * freq_hz)) - 1
        prescale = max(3, min(255, prescale))

        with self._lock:
            # Read current MODE1
            old_mode = self._bus.read_byte_data(self._address, self._REG_MODE1)

            # Enter sleep mode (clear restart bit, set sleep bit)
            sleep_mode = (old_mode & 0x7F) | self._MODE1_SLEEP
            self._bus.write_byte_data(self._address, self._REG_MODE1, sleep_mode)

            # Write prescale (only writable in sleep mode)
            self._bus.write_byte_data(self._address, self._REG_PRESCALE, prescale)

            # Wake up (clear sleep bit, keep other bits)
            wake_mode = old_mode & 0x7F  # Clear restart bit temporarily
            self._bus.write_byte_data(self._address, self._REG_MODE1, wake_mode)
            time.sleep(0.005)  # Wait for oscillator (500us min per datasheet)

            # Set restart bit to resume PWM output
            self._bus.write_byte_data(
                self._address, self._REG_MODE1, wake_mode | self._MODE1_RESTART
            )

        actual_freq = self._OSC_CLOCK_HZ / (4096.0 * (prescale + 1))
        logger.info(
            "PCA9685 frequency set to %d Hz (prescale=%d, actual=%.1f Hz) on bus %d",
            freq_hz, prescale, actual_freq, self._actual_bus_num
        )

    def set_duty_cycle(self, channel: int, duty_cycle: int) -> None:
        """Set 16-bit duty cycle on a specific channel. Thread-safe.

        Converts the 16-bit duty cycle value (Adafruit convention) to the
        PCA9685's native 12-bit ON/OFF counter registers.

        Conversion: 12bit = (16bit + 1) >> 4
        Special cases:
          - 0xFFFF (65535) -> full ON  (LED always on)
          - 0x0000 (0)     -> full OFF (LED always off)

        Register layout per channel (4 bytes):
          LEDn_ON_L  (base + 0): ON count low byte
          LEDn_ON_H  (base + 1): ON count high byte (bit 4 = full ON)
          LEDn_OFF_L (base + 2): OFF count low byte
          LEDn_OFF_H (base + 3): OFF count high byte (bit 4 = full OFF)

        For normal PWM: ON=0, OFF=duty_value (pulse starts at count 0,
        ends at count duty_value within each 4096-count cycle).

        Args:
            channel: PCA9685 channel number (0-15).
            duty_cycle: 16-bit duty cycle value (0-65535).

        Raises:
            ValueError: If channel or duty_cycle out of valid range.
            RuntimeError: If not initialized.
        """
        if not self._initialized:
            raise RuntimeError("PCA9685 not initialized. Call initialize() first.")
        if not 0 <= channel < self.NUM_CHANNELS:
            raise ValueError(f"Channel {channel} out of range (0-{self.NUM_CHANNELS - 1})")
        if not 0 <= duty_cycle <= self.MAX_DUTY_CYCLE:
            raise ValueError(f"Duty cycle {duty_cycle} out of range (0-{self.MAX_DUTY_CYCLE})")

        # Convert 16-bit to PCA9685 register values
        if duty_cycle == 0xFFFF:
            # Full ON: set bit 4 of ON_H register
            on = 0x1000
            off = 0
        elif duty_cycle == 0:
            # Full OFF: set bit 4 of OFF_H register
            on = 0
            off = 0x1000
        else:
            # Normal PWM: ON at count 0, OFF at converted count
            on = 0
            off = (duty_cycle + 1) >> 4  # 16-bit -> 12-bit

        # Calculate register base address for this channel
        reg_base = self._REG_LED0_ON_L + 4 * channel

        with self._lock:
            try:
                # Write all 4 bytes in one I2C transaction (auto-increment enabled)
                self._bus.write_i2c_block_data(
                    self._address, reg_base,
                    [on & 0xFF, (on >> 8) & 0x1F,
                     off & 0xFF, (off >> 8) & 0x1F]
                )
            except Exception as e:
                logger.error(
                    "I2C write failed on channel %d (bus %d): %s",
                    channel, self._actual_bus_num, e
                )
                raise

    def set_all_neutral(self, neutral_duty: int = 4915) -> None:
        """Set all 16 channels to neutral duty cycle.

        Uses the PCA9685's ALL_LED registers to write all channels in a
        single I2C transaction (faster than writing channels individually).
        Falls back to per-channel writes if the bulk write fails.

        Used for emergency stop and shutdown. Sets every channel,
        not just the motor channels, as a safety measure.

        Args:
            neutral_duty: Duty cycle for neutral position.
                          Default 4915 = 1500us at 50Hz = Sabertooth stopped.
        """
        if not self._initialized:
            raise RuntimeError("PCA9685 not initialized. Call initialize() first.")

        # Convert 16-bit to 12-bit
        off = (neutral_duty + 1) >> 4

        # Try bulk write first (single I2C transaction for all channels)
        try:
            with self._lock:
                self._bus.write_i2c_block_data(
                    self._address, self._REG_ALL_LED_ON_L,
                    [0, 0, off & 0xFF, (off >> 8) & 0x1F]
                )
            return
        except Exception as e:
            logger.error(
                "Bulk neutral write failed (bus %d): %s. Trying per-channel.",
                self._actual_bus_num, e
            )

        # Fallback: write each channel individually
        for ch in range(self.NUM_CHANNELS):
            try:
                self.set_duty_cycle(ch, neutral_duty)
            except Exception as e:
                logger.error("Failed to set channel %d to neutral: %s", ch, e)

    def shutdown(self) -> None:
        """Set all channels to neutral and close the I2C bus.

        Safe to call multiple times. Always attempts to set neutral
        before closing the connection, so motors stop even if errors occur.

        IMPORTANT: Called during node shutdown. Motors MUST return to
        neutral (1500us = stopped) before we release the I2C bus,
        otherwise the PCA9685 holds the last PWM value indefinitely.
        """
        if self._initialized:
            try:
                self.set_all_neutral()
            except Exception as e:
                logger.error("Error setting neutral during shutdown: %s", e)

            self._initialized = False

        self._cleanup_bus()
        logger.info("PCA9685 (smbus2) shutdown complete")

    def _cleanup_bus(self) -> None:
        """Close the smbus2 connection if open."""
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception as e:
                logger.debug("Error closing I2C bus: %s", e)
            self._bus = None

    def is_initialized(self) -> bool:
        """Return True if hardware is initialized and ready."""
        return self._initialized
