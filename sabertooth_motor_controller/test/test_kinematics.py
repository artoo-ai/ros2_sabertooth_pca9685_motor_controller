"""Unit tests for drive kinematics.

Tests the TankDriveKinematics class that converts geometry_msgs/Twist
(linear.x, angular.z) into left/right motor speeds (-1.0 to +1.0).

These tests verify:
  - Zero input produces zero output
  - Pure forward/reverse movement
  - Pure rotation (pivot turns)
  - Combined movement with turning
  - Normalization preserves turning ratio
  - Output is always within -1.0 to +1.0
  - Symmetry properties

Run with: pytest test/test_kinematics.py -v
"""

import pytest
from sabertooth_motor_controller.drive_kinematics import (
    TankDriveKinematics,
    create_kinematics,
)


@pytest.fixture
def kinematics():
    """Standard tank drive kinematics for testing."""
    return TankDriveKinematics(
        wheel_separation_m=0.45,
        wheel_radius_m=0.075,
        max_linear_speed_ms=1.0,
        max_angular_speed_rads=2.0,
    )


class TestZeroInput:
    """Test that zero Twist produces zero motor output."""

    def test_zero_twist(self, kinematics):
        left, right = kinematics.twist_to_motor_speeds(0.0, 0.0)
        assert left == 0.0
        assert right == 0.0


class TestPureLinear:
    """Test pure forward/backward movement (no turning)."""

    def test_forward(self, kinematics):
        """Forward: both motors should be positive and equal."""
        left, right = kinematics.twist_to_motor_speeds(0.5, 0.0)
        assert left > 0.0
        assert right > 0.0
        assert abs(left - right) < 1e-9  # Should be equal

    def test_reverse(self, kinematics):
        """Reverse: both motors should be negative and equal."""
        left, right = kinematics.twist_to_motor_speeds(-0.5, 0.0)
        assert left < 0.0
        assert right < 0.0
        assert abs(left - right) < 1e-9

    def test_full_forward(self, kinematics):
        """Full speed forward at max_linear_speed."""
        left, right = kinematics.twist_to_motor_speeds(1.0, 0.0)
        assert left > 0.0
        assert right > 0.0
        assert abs(left - right) < 1e-9


class TestPureRotation:
    """Test pure rotation (pivot turns) with no forward movement."""

    def test_turn_left_ccw(self, kinematics):
        """Positive angular.z = CCW = turn left.
        Left motor goes backward, right motor goes forward."""
        left, right = kinematics.twist_to_motor_speeds(0.0, 1.0)
        assert left < 0.0   # Left wheel backward
        assert right > 0.0  # Right wheel forward

    def test_turn_right_cw(self, kinematics):
        """Negative angular.z = CW = turn right.
        Left motor goes forward, right motor goes backward."""
        left, right = kinematics.twist_to_motor_speeds(0.0, -1.0)
        assert left > 0.0   # Left wheel forward
        assert right < 0.0  # Right wheel backward


class TestCombinedMovement:
    """Test forward/backward movement combined with turning."""

    def test_forward_left_turn(self, kinematics):
        """Forward + left turn: right faster than left."""
        left, right = kinematics.twist_to_motor_speeds(0.5, 0.5)
        assert right > left  # Right wheel faster for left turn

    def test_forward_right_turn(self, kinematics):
        """Forward + right turn: left faster than right."""
        left, right = kinematics.twist_to_motor_speeds(0.5, -0.5)
        assert left > right  # Left wheel faster for right turn


class TestNormalization:
    """Test that output normalization preserves turning ratio."""

    def test_normalization_preserves_ratio(self, kinematics):
        """When one motor would exceed 1.0, both scale proportionally."""
        # Use extreme values that would exceed 1.0 before normalization
        left, right = kinematics.twist_to_motor_speeds(1.0, 2.0)
        # Both should be within range
        assert -1.0 <= left <= 1.0
        assert -1.0 <= right <= 1.0
        # At least one should be at the limit
        assert abs(left) == pytest.approx(1.0, abs=0.01) or \
               abs(right) == pytest.approx(1.0, abs=0.01)


class TestOutputRange:
    """Test that output is always within valid range."""

    @pytest.mark.parametrize("linear,angular", [
        (0.0, 0.0),
        (1.0, 0.0), (-1.0, 0.0),
        (0.0, 2.0), (0.0, -2.0),
        (1.0, 2.0), (-1.0, -2.0),
        (1.0, -2.0), (-1.0, 2.0),
        (0.1, 0.1), (100.0, 100.0),  # Extreme values
    ])
    def test_output_clamped(self, kinematics, linear, angular):
        """Output is always within -1.0 to +1.0 regardless of input."""
        left, right = kinematics.twist_to_motor_speeds(linear, angular)
        assert -1.0 <= left <= 1.0, f"Left {left} out of range for ({linear}, {angular})"
        assert -1.0 <= right <= 1.0, f"Right {right} out of range for ({linear}, {angular})"


class TestSymmetry:
    """Test symmetry properties of the kinematics."""

    def test_angular_symmetry(self, kinematics):
        """Negating angular.z should swap left and right speeds."""
        left1, right1 = kinematics.twist_to_motor_speeds(0.5, 0.3)
        left2, right2 = kinematics.twist_to_motor_speeds(0.5, -0.3)
        assert left1 == pytest.approx(right2, abs=1e-9)
        assert right1 == pytest.approx(left2, abs=1e-9)

    def test_linear_symmetry(self, kinematics):
        """Negating linear.x with same angular.z: left_fwd == -right_rev.

        In differential drive with angular != 0:
          left(v,w)  = v - w*d/2    right(v,w)  = v + w*d/2
          left(-v,w) = -v - w*d/2   right(-v,w) = -v + w*d/2
        So left(v,w) == -right(-v,w) and right(v,w) == -left(-v,w).
        """
        left1, right1 = kinematics.twist_to_motor_speeds(0.5, 0.3)
        left2, right2 = kinematics.twist_to_motor_speeds(-0.5, 0.3)
        assert left1 == pytest.approx(-right2, abs=1e-9)
        assert right1 == pytest.approx(-left2, abs=1e-9)


class TestValidation:
    """Test constructor validation."""

    def test_invalid_wheel_separation(self):
        with pytest.raises(ValueError):
            TankDriveKinematics(wheel_separation_m=0.0)

    def test_negative_wheel_separation(self):
        with pytest.raises(ValueError):
            TankDriveKinematics(wheel_separation_m=-1.0)

    def test_invalid_max_linear(self):
        with pytest.raises(ValueError):
            TankDriveKinematics(max_linear_speed_ms=0.0)


class TestFactory:
    """Test the kinematics factory function."""

    def test_create_tank(self):
        k = create_kinematics("tank", wheel_separation_m=0.5)
        assert isinstance(k, TankDriveKinematics)

    def test_unknown_type(self):
        with pytest.raises(ValueError):
            create_kinematics("unknown_drive_type")
