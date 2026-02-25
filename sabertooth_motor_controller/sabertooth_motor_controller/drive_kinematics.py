"""Drive kinematics module - converts Twist commands to motor speeds.

Converts geometry_msgs/Twist (linear.x, angular.z) into per-motor speed
commands for different drive configurations. Uses a Protocol interface
so drive types can be swapped easily.

Currently implemented:
  - TankDriveKinematics: Standard differential/tank drive (2 motors)

Future possibilities (just implement the DriveKinematics protocol):
  - MecanumKinematics: 4-wheel omnidirectional
  - AckermannKinematics: Car-like steering
  - SkidSteerKinematics: 4-wheel skid steer

Switching drive types:
  1. Create a new class implementing the DriveKinematics protocol
  2. Change the 'kinematics.type' ROS parameter
  3. The node will instantiate the correct kinematics class

The math is pure Python with no ROS dependencies, making it easy to
unit test and reuse outside of ROS.
"""

from typing import Protocol, Tuple


class DriveKinematics(Protocol):
    """Interface that all kinematics implementations must follow.

    Any class with a twist_to_motor_speeds method matching this signature
    can be used as a drop-in kinematics implementation.
    """

    def twist_to_motor_speeds(
        self, linear_x: float, angular_z: float
    ) -> Tuple[float, float]:
        """Convert Twist velocities to normalized motor speeds.

        Args:
            linear_x: Forward/backward velocity (m/s). Positive = forward.
            angular_z: Rotational velocity (rad/s). Positive = counter-clockwise
                       (left turn when viewed from above).

        Returns:
            Tuple of (left_speed, right_speed), each normalized to -1.0..+1.0.
        """
        ...


class TankDriveKinematics:
    """Differential (tank) drive kinematics.

    Converts linear.x and angular.z from /cmd_vel into left and right
    motor speeds using standard differential drive equations:

        v_left  = linear_x - (angular_z * wheel_separation / 2)
        v_right = linear_x + (angular_z * wheel_separation / 2)

    The raw velocities are then normalized to -1.0..+1.0 based on the
    configured maximum speeds. If either wheel would exceed 1.0, both
    are scaled down proportionally to preserve the turning radius.

    Example:
        kinematics = TankDriveKinematics(
            wheel_separation_m=0.45,
            max_linear_speed_ms=1.0,
            max_angular_speed_rads=2.0,
        )
        left, right = kinematics.twist_to_motor_speeds(0.5, 0.3)
    """

    def __init__(
        self,
        wheel_separation_m: float = 0.45,
        wheel_radius_m: float = 0.075,
        max_linear_speed_ms: float = 1.0,
        max_angular_speed_rads: float = 2.0,
    ):
        """
        Args:
            wheel_separation_m: Distance between left and right wheel centers
                                in meters. Measure from center-to-center of
                                the drive wheels/tracks.
            wheel_radius_m: Wheel radius in meters. Used for velocity scaling
                            when computing motor RPM relationships.
            max_linear_speed_ms: Maximum linear speed (m/s) that maps to
                                 motor speed 1.0. Determines how sensitive
                                 the robot is to forward/backward commands.
            max_angular_speed_rads: Maximum angular speed (rad/s) that maps
                                    to full differential. Determines turning
                                    sensitivity.
        """
        if wheel_separation_m <= 0:
            raise ValueError(f"wheel_separation_m must be positive, got {wheel_separation_m}")
        if max_linear_speed_ms <= 0:
            raise ValueError(f"max_linear_speed_ms must be positive, got {max_linear_speed_ms}")
        if max_angular_speed_rads <= 0:
            raise ValueError(f"max_angular_speed_rads must be positive, got {max_angular_speed_rads}")

        self._wheel_sep = wheel_separation_m
        self._wheel_radius = wheel_radius_m
        self._max_linear = max_linear_speed_ms
        self._max_angular = max_angular_speed_rads

    def twist_to_motor_speeds(
        self, linear_x: float, angular_z: float
    ) -> Tuple[float, float]:
        """Convert Twist to normalized motor speeds.

        Processing steps:
        1. Clamp input velocities to configured maximums
        2. Compute differential wheel velocities
        3. Normalize to -1.0..+1.0 range
        4. If either wheel exceeds 1.0, scale both proportionally
           (this preserves the turning radius)

        Args:
            linear_x: Forward velocity from Twist.linear.x (m/s).
                      Positive = forward, negative = backward.
            angular_z: Rotation from Twist.angular.z (rad/s).
                       Positive = turn left (CCW from above).

        Returns:
            (left_speed, right_speed) each in range -1.0 to +1.0.
        """
        # Clamp inputs to configured maximums
        linear_x = max(-self._max_linear, min(self._max_linear, linear_x))
        angular_z = max(-self._max_angular, min(self._max_angular, angular_z))

        # Differential drive equations:
        # v_left  = linear_x - (angular_z * wheel_separation / 2)
        # v_right = linear_x + (angular_z * wheel_separation / 2)
        #
        # angular_z positive = CCW = turn left:
        #   left wheel slows down, right wheel speeds up
        half_sep = self._wheel_sep / 2.0
        v_left = linear_x - (angular_z * half_sep)
        v_right = linear_x + (angular_z * half_sep)

        # Normalize to -1.0..+1.0 using max achievable wheel velocity
        # Max wheel velocity occurs at max_linear + max_angular * half_sep
        max_wheel_vel = self._max_linear + (self._max_angular * half_sep)
        if max_wheel_vel > 0:
            left_norm = v_left / max_wheel_vel
            right_norm = v_right / max_wheel_vel
        else:
            left_norm = 0.0
            right_norm = 0.0

        # Scale both if either exceeds 1.0 (preserves turning radius)
        return self._normalize_and_scale(left_norm, right_norm)

    @staticmethod
    def _normalize_and_scale(left: float, right: float) -> Tuple[float, float]:
        """Normalize speeds: if either exceeds 1.0, scale both proportionally.

        This preserves the turning radius when one motor would exceed
        maximum speed. Without this, large turns at high speed would
        clip one motor and change the turning behavior.

        Args:
            left: Left motor speed (may exceed -1.0..+1.0)
            right: Right motor speed (may exceed -1.0..+1.0)

        Returns:
            (left, right) both within -1.0..+1.0, ratio preserved.
        """
        max_abs = max(abs(left), abs(right))
        if max_abs > 1.0:
            left /= max_abs
            right /= max_abs
        return (
            max(-1.0, min(1.0, left)),
            max(-1.0, min(1.0, right)),
        )


def create_kinematics(
    kinematics_type: str = "tank",
    **kwargs,
) -> DriveKinematics:
    """Factory function to create kinematics instance from config.

    Args:
        kinematics_type: Drive type string ("tank", or future types).
        **kwargs: Keyword arguments passed to the kinematics constructor.

    Returns:
        A DriveKinematics instance.

    Raises:
        ValueError: If kinematics_type is not recognized.
    """
    if kinematics_type == "tank":
        return TankDriveKinematics(**kwargs)
    else:
        raise ValueError(
            f"Unknown kinematics type '{kinematics_type}'. "
            f"Available types: 'tank'"
        )
