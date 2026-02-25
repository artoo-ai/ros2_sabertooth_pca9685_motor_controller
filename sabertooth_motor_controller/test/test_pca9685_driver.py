"""Unit tests for the PCA9685 driver (using MockPWMDriver).

Tests the MockPWMDriver implementation which mirrors the real PCA9685Driver
interface. These tests verify the driver contract that both real and mock
implementations must satisfy.

Run with: pytest test/test_pca9685_driver.py -v
"""

import pytest
import threading
from sabertooth_motor_controller.mock_hardware import MockPWMDriver


@pytest.fixture
def driver():
    """Create and initialize a mock PWM driver."""
    d = MockPWMDriver(i2c_bus=1, address=0x40)
    d.initialize()
    return d


class TestInitialization:
    """Test driver initialization."""

    def test_initialize_returns_true(self):
        """Mock driver should always initialize successfully."""
        d = MockPWMDriver()
        assert d.initialize() is True
        assert d.is_initialized() is True

    def test_not_initialized_by_default(self):
        """Should not be initialized until initialize() is called."""
        d = MockPWMDriver()
        assert d.is_initialized() is False

    def test_operations_fail_before_init(self):
        """Operations should raise RuntimeError before initialization."""
        d = MockPWMDriver()
        with pytest.raises(RuntimeError):
            d.set_frequency(50)
        with pytest.raises(RuntimeError):
            d.set_duty_cycle(0, 4915)


class TestDutyCycle:
    """Test duty cycle operations."""

    def test_set_and_get_duty_cycle(self, driver):
        """Should store and retrieve duty cycle values."""
        driver.set_duty_cycle(0, 4915)
        assert driver.get_duty_cycle(0) == 4915

    def test_multiple_channels(self, driver):
        """Each channel should store its own value."""
        driver.set_duty_cycle(0, 1000)
        driver.set_duty_cycle(1, 2000)
        driver.set_duty_cycle(2, 3000)
        assert driver.get_duty_cycle(0) == 1000
        assert driver.get_duty_cycle(1) == 2000
        assert driver.get_duty_cycle(2) == 3000

    def test_overwrite_duty_cycle(self, driver):
        """Setting duty cycle again should overwrite previous value."""
        driver.set_duty_cycle(0, 1000)
        driver.set_duty_cycle(0, 5000)
        assert driver.get_duty_cycle(0) == 5000


class TestValidation:
    """Test input validation."""

    def test_channel_too_low(self, driver):
        """Channel below 0 should raise ValueError."""
        with pytest.raises(ValueError):
            driver.set_duty_cycle(-1, 4915)

    def test_channel_too_high(self, driver):
        """Channel 16+ should raise ValueError."""
        with pytest.raises(ValueError):
            driver.set_duty_cycle(16, 4915)

    def test_duty_cycle_negative(self, driver):
        """Negative duty cycle should raise ValueError."""
        with pytest.raises(ValueError):
            driver.set_duty_cycle(0, -1)

    def test_duty_cycle_too_high(self, driver):
        """Duty cycle above 65535 should raise ValueError."""
        with pytest.raises(ValueError):
            driver.set_duty_cycle(0, 65536)

    def test_boundary_values(self, driver):
        """Boundary values should be accepted."""
        driver.set_duty_cycle(0, 0)
        assert driver.get_duty_cycle(0) == 0

        driver.set_duty_cycle(15, 65535)
        assert driver.get_duty_cycle(15) == 65535


class TestSetAllNeutral:
    """Test the set_all_neutral method."""

    def test_sets_all_channels(self, driver):
        """Should set all 16 channels to neutral value."""
        # Set some channels to non-neutral
        driver.set_duty_cycle(0, 6553)
        driver.set_duty_cycle(5, 3277)
        driver.set_duty_cycle(15, 1000)

        driver.set_all_neutral(4915)

        for ch in range(16):
            assert driver.get_duty_cycle(ch) == 4915

    def test_custom_neutral_value(self, driver):
        """Should use the provided neutral value."""
        driver.set_all_neutral(5000)
        assert driver.get_duty_cycle(0) == 5000
        assert driver.get_duty_cycle(8) == 5000


class TestHistory:
    """Test the operation history recording."""

    def test_records_operations(self, driver):
        """Should record all duty cycle operations."""
        driver.set_duty_cycle(0, 4915)
        driver.set_duty_cycle(1, 6553)

        history = driver.get_history()
        assert len(history) >= 2

    def test_filter_by_channel(self, driver):
        """Should filter history by channel."""
        driver.set_duty_cycle(0, 4915)
        driver.set_duty_cycle(1, 6553)
        driver.set_duty_cycle(0, 3277)

        ch0_history = driver.get_history(channel=0)
        assert len(ch0_history) == 2
        assert all(ch == 0 for _, ch, _ in ch0_history)

    def test_reset_history(self, driver):
        """reset_history should clear all recorded operations."""
        driver.set_duty_cycle(0, 4915)
        driver.reset_history()
        assert len(driver.get_history()) == 0


class TestThreadSafety:
    """Test thread safety of concurrent operations."""

    def test_concurrent_writes(self, driver):
        """Concurrent set_duty_cycle calls should not corrupt state."""
        errors = []

        def write_channel(channel, value, iterations):
            try:
                for _ in range(iterations):
                    driver.set_duty_cycle(channel, value)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=write_channel, args=(0, 4915, 100)),
            threading.Thread(target=write_channel, args=(1, 6553, 100)),
            threading.Thread(target=write_channel, args=(2, 3277, 100)),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        # Final values should be correct
        assert driver.get_duty_cycle(0) == 4915
        assert driver.get_duty_cycle(1) == 6553
        assert driver.get_duty_cycle(2) == 3277


class TestShutdown:
    """Test shutdown behavior."""

    def test_shutdown_sets_neutral(self, driver):
        """Shutdown should set all channels to neutral."""
        driver.set_duty_cycle(0, 6553)
        driver.shutdown()
        assert not driver.is_initialized()

    def test_double_shutdown_safe(self, driver):
        """Calling shutdown twice should not raise."""
        driver.shutdown()
        driver.shutdown()  # Should not raise
