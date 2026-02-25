"""Unit tests for the Sabertooth PWM driver.

Tests the SabertoothDriver class that converts normalized speeds (-1.0 to +1.0)
into PCA9685 duty cycle values for Sabertooth RC mode operation.

Key values at 50Hz (period=20000us, 16-bit resolution=65535):
  - Neutral (1500us): duty_cycle = 4915
  - Full forward (2000us): duty_cycle = 6553
  - Full reverse (1000us): duty_cycle = 3277

Run with: pytest test/test_sabertooth_driver.py -v
"""

import pytest
from sabertooth_motor_controller.sabertooth_driver import SabertoothDriver
from sabertooth_motor_controller.mock_hardware import MockPWMDriver


@pytest.fixture
def mock_pwm():
    """Create a mock PWM driver."""
    driver = MockPWMDriver()
    driver.initialize()
    return driver


@pytest.fixture
def sabertooth(mock_pwm):
    """Create a SabertoothDriver with mock PWM backend."""
    driver = SabertoothDriver(
        pwm_driver=mock_pwm,
        left_channel=0,
        right_channel=1,
    )
    driver.initialize()
    return driver


class TestPulseConversions:
    """Test the mathematical conversion from speed to pulse to duty cycle."""

    def test_neutral_speed(self, sabertooth):
        """Speed 0.0 should produce 1500us pulse."""
        pulse = sabertooth.speed_to_pulse_us(0.0)
        assert pulse == pytest.approx(1500.0, abs=1.0)

    def test_full_forward_speed(self, sabertooth):
        """Speed 1.0 should produce 2000us pulse."""
        pulse = sabertooth.speed_to_pulse_us(1.0)
        assert pulse == pytest.approx(2000.0, abs=1.0)

    def test_full_reverse_speed(self, sabertooth):
        """Speed -1.0 should produce 1000us pulse."""
        pulse = sabertooth.speed_to_pulse_us(-1.0)
        assert pulse == pytest.approx(1000.0, abs=1.0)

    def test_half_forward(self, sabertooth):
        """Speed 0.5 should produce 1750us pulse."""
        pulse = sabertooth.speed_to_pulse_us(0.5)
        assert pulse == pytest.approx(1750.0, abs=1.0)

    def test_half_reverse(self, sabertooth):
        """Speed -0.5 should produce 1250us pulse."""
        pulse = sabertooth.speed_to_pulse_us(-0.5)
        assert pulse == pytest.approx(1250.0, abs=1.0)


class TestDutyCycleConversions:
    """Test pulse width to PCA9685 duty cycle conversion."""

    def test_neutral_duty_cycle(self, sabertooth):
        """1500us at 50Hz should produce duty_cycle ~4915."""
        duty = sabertooth.pulse_us_to_duty_cycle(1500.0)
        assert duty == pytest.approx(4915, abs=5)

    def test_full_forward_duty_cycle(self, sabertooth):
        """2000us at 50Hz should produce duty_cycle ~6553."""
        duty = sabertooth.pulse_us_to_duty_cycle(2000.0)
        assert duty == pytest.approx(6553, abs=5)

    def test_full_reverse_duty_cycle(self, sabertooth):
        """1000us at 50Hz should produce duty_cycle ~3277."""
        duty = sabertooth.pulse_us_to_duty_cycle(1000.0)
        assert duty == pytest.approx(3277, abs=5)


class TestMotorOutput:
    """Test that set_motors sends correct duty cycles to PCA9685."""

    def test_neutral_output(self, sabertooth, mock_pwm):
        """Speed 0.0 should send neutral duty cycle to both channels."""
        sabertooth.set_motors(0.0, 0.0)
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        assert left_duty == pytest.approx(4915, abs=5)
        assert right_duty == pytest.approx(4915, abs=5)

    def test_forward_output(self, sabertooth, mock_pwm):
        """Positive speed should produce duty cycle above neutral."""
        sabertooth.set_motors(0.5, 0.5)
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        assert left_duty > 4915
        assert right_duty > 4915

    def test_reverse_output(self, sabertooth, mock_pwm):
        """Negative speed should produce duty cycle below neutral."""
        sabertooth.set_motors(-0.5, -0.5)
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        assert left_duty < 4915
        assert right_duty < 4915

    def test_differential_output(self, sabertooth, mock_pwm):
        """Different speeds should produce different duty cycles."""
        sabertooth.set_motors(0.5, -0.3)
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        assert left_duty != right_duty
        assert left_duty > 4915   # Forward
        assert right_duty < 4915  # Reverse


class TestSpeedClamping:
    """Test that speeds beyond -1.0 to +1.0 are clamped."""

    def test_clamp_above_one(self, sabertooth):
        """Speed > 1.0 should clamp to 2000us pulse."""
        pulse = sabertooth.speed_to_pulse_us(1.5)
        assert pulse == pytest.approx(2000.0, abs=1.0)

    def test_clamp_below_neg_one(self, sabertooth):
        """Speed < -1.0 should clamp to 1000us pulse."""
        pulse = sabertooth.speed_to_pulse_us(-1.5)
        assert pulse == pytest.approx(1000.0, abs=1.0)


class TestDeadband:
    """Test that small speeds within deadband snap to neutral."""

    def test_small_positive_deadband(self, sabertooth, mock_pwm):
        """Very small positive speed within deadband -> neutral."""
        sabertooth.set_motors(0.01, 0.01)
        left_duty = mock_pwm.get_duty_cycle(0)
        neutral_duty = sabertooth.get_neutral_duty_cycle()
        assert left_duty == neutral_duty

    def test_small_negative_deadband(self, sabertooth, mock_pwm):
        """Very small negative speed within deadband -> neutral."""
        sabertooth.set_motors(-0.01, -0.01)
        left_duty = mock_pwm.get_duty_cycle(0)
        neutral_duty = sabertooth.get_neutral_duty_cycle()
        assert left_duty == neutral_duty

    def test_above_deadband(self, sabertooth, mock_pwm):
        """Speed above deadband should NOT be at neutral."""
        sabertooth.set_motors(0.2, 0.2)
        left_duty = mock_pwm.get_duty_cycle(0)
        neutral_duty = sabertooth.get_neutral_duty_cycle()
        assert left_duty != neutral_duty


class TestMotorInversion:
    """Test motor direction inversion."""

    def test_left_inverted(self, mock_pwm):
        """With left_inverted=True, positive speed sends reverse pulse."""
        driver = SabertoothDriver(
            pwm_driver=mock_pwm,
            left_channel=0,
            right_channel=1,
            left_inverted=True,
        )
        driver.initialize()
        driver.set_motors(0.5, 0.5)
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        # Left should be below neutral (inverted), right above
        assert left_duty < 4915
        assert right_duty > 4915


class TestSetNeutral:
    """Test the set_neutral method."""

    def test_set_neutral_sends_correct_duty(self, sabertooth, mock_pwm):
        """set_neutral should send neutral duty cycle to both channels."""
        # First set to non-neutral
        sabertooth.set_motors(0.5, 0.5)
        # Then set neutral
        sabertooth.set_neutral()
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        neutral = sabertooth.get_neutral_duty_cycle()
        assert left_duty == neutral
        assert right_duty == neutral


class TestShutdown:
    """Test shutdown behavior."""

    def test_shutdown_sends_neutral(self, sabertooth, mock_pwm):
        """shutdown() should set neutral before closing."""
        sabertooth.set_motors(1.0, 1.0)
        sabertooth.shutdown()
        # After shutdown, channels should be at neutral
        left_duty = mock_pwm.get_duty_cycle(0)
        right_duty = mock_pwm.get_duty_cycle(1)
        # Note: mock_pwm.shutdown() sets all channels to neutral (4915)
        assert left_duty == pytest.approx(4915, abs=5)
