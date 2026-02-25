# Direct motor speed command
# Bypasses kinematics - sends speed directly to each motor
#
# Use /cmd_vel (geometry_msgs/Twist) for standard navigation commands.
# Use this message for direct motor control when you need explicit
# left/right motor speed values.

std_msgs/Header header

# Motor speeds: -1.0 (full reverse) to +1.0 (full forward)
float32 left_speed
float32 right_speed

# Source identification for command arbitration priority
uint8 SOURCE_AUTONOMOUS = 0
uint8 SOURCE_TELEOP = 1
uint8 SOURCE_RC = 2
uint8 SOURCE_ESTOP = 3
uint8 source
