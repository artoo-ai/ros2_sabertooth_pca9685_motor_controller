# Motor controller status message
# Published periodically for monitoring and diagnostics
#
# Subscribe to /motor_status to monitor the controller's state,
# current motor speeds, safety status, and RC input values.

std_msgs/Header header

# Current motor speeds
float32 left_speed_commanded      # Last commanded left speed (-1.0 to 1.0)
float32 right_speed_commanded     # Last commanded right speed (-1.0 to 1.0)
float32 left_speed_actual         # After ramp/safety limiting (-1.0 to 1.0)
float32 right_speed_actual        # After ramp/safety limiting (-1.0 to 1.0)
uint16 left_pwm_duty_cycle        # Raw 16-bit duty cycle sent to PCA9685
uint16 right_pwm_duty_cycle       # Raw 16-bit duty cycle sent to PCA9685

# Safety state
uint8 STATE_NORMAL = 0
uint8 STATE_TIMEOUT = 1
uint8 STATE_ESTOP = 2
uint8 STATE_RC_OVERRIDE = 3
uint8 STATE_INITIALIZING = 4
uint8 STATE_ERROR = 5
uint8 safety_state

# Active command source (same constants as MotorCommand.source)
uint8 active_source

# Status flags
bool hardware_present             # True if PCA9685 detected on I2C
bool rc_connected                 # True if RC signals being received
bool estop_active                 # True if e-stop is engaged
bool command_timeout              # True if no commands within timeout
bool speed_limited                # True if speed was clamped by limiter

# Timing
float64 last_command_age_sec      # Seconds since last valid command
float64 uptime_sec                # Node uptime in seconds

# RC input diagnostics (raw pulse widths in microseconds)
float32 rc_ch1_pulse_us
float32 rc_ch2_pulse_us
float32 rc_ch3_pulse_us
