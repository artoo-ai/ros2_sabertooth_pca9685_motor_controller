"""Main ROS2 node for the Sabertooth motor controller.

This node is the central orchestrator that:
  1. Subscribes to /cmd_vel (geometry_msgs/Twist) and /motor_cmd (MotorCommand)
  2. Reads RC receiver input for override control
  3. Runs the safety monitor state machine
  4. Sends processed commands to the Sabertooth via PCA9685
  5. Publishes /motor_status diagnostics
  6. Provides /estop and /reset services

CONTROL LOOP ARCHITECTURE (50Hz):
  Each cycle:
    Read RC input -> Determine active source -> Get target speeds ->
    Safety processing (timeout, speed limit, accel limit) ->
    Output to hardware -> Publish status

COMMAND PRIORITY (highest first):
  E-STOP > RC Override > Teleop > Autonomous

SIMULATION MODE:
  When running on MacBook or without PCA9685 hardware, the node
  automatically falls back to mock drivers. All ROS2 topics and
  services work identically - only the hardware output is simulated.

USAGE:
  # Start with hardware (on Jetson):
  ros2 launch sabertooth_motor_controller motor_controller.launch.py

  # Start in simulation mode (on Mac):
  ros2 launch sabertooth_motor_controller motor_controller.launch.py simulation_mode:=true

  # Teleop with keyboard:
  ros2 launch sabertooth_motor_controller teleop.launch.py simulation_mode:=true
"""

import sys
import platform
import logging
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

# Import our modules
from .sabertooth_driver import SabertoothDriver
from .pca9685_driver import PCA9685Driver
from .mock_hardware import MockPWMDriver, MockRCInput
from .drive_kinematics import TankDriveKinematics, create_kinematics
from .safety_monitor import SafetyMonitor, SafetyState, CommandSource
from .rc_input import RCInput

logger = logging.getLogger(__name__)


class SabertoothMotorControllerNode(Node):
    """ROS2 node for Sabertooth motor controller via PCA9685 PWM.

    This node manages the complete motor control pipeline from
    receiving commands to outputting PWM signals.

    Lifecycle:
      1. __init__: Declare parameters, create subscriptions/publishers/timers
      2. _initialize_hardware: Create driver instances (real or mock)
      3. _control_loop (50Hz timer): Main control cycle
      4. destroy_node: Set neutral, cleanup
    """

    NODE_NAME = "sabertooth_motor_controller"
    CONTROL_LOOP_HZ = 50.0  # Match Sabertooth PWM frequency

    def __init__(self):
        super().__init__(self.NODE_NAME)

        # Declare all ROS2 parameters
        self._declare_all_parameters()

        # Read parameters into instance variables
        self._read_parameters()

        # Detect platform for auto simulation mode
        self._sim_mode = self._param_sim_mode
        if not self._sim_mode:
            self._sim_mode = not self._is_jetson_platform()

        # Initialize hardware (real or mock)
        self._initialize_hardware()

        # Create kinematics
        self._kinematics = create_kinematics(
            kinematics_type=self._param_kinematics_type,
            wheel_separation_m=self._param_wheel_sep,
            wheel_radius_m=self._param_wheel_radius,
            max_linear_speed_ms=self._param_max_linear,
            max_angular_speed_rads=self._param_max_angular,
        )

        # Create safety monitor
        self._safety = SafetyMonitor(
            command_timeout_ms=self._param_cmd_timeout_ms,
            heartbeat_timeout_ms=self._param_heartbeat_timeout_ms,
            max_speed_teleop=self._param_max_speed_teleop,
            max_speed_autonomous=self._param_max_speed_auto,
            max_speed_rc=self._param_max_speed_rc,
            accel_limit=self._param_accel_limit,
            decel_limit=self._param_decel_limit,
            emergency_decel_limit=self._param_emergency_decel,
            startup_delay_sec=self._param_startup_delay,
            on_state_change=self._on_safety_state_change,
        )

        # Pending command state
        self._pending_left = 0.0
        self._pending_right = 0.0
        self._pending_source = CommandSource.AUTONOMOUS
        self._last_loop_time = time.monotonic()

        # Last output state for status publishing
        self._last_left_duty = 0
        self._last_right_duty = 0
        self._last_was_limited = False
        self._software_estop = False

        # Set up ROS2 interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        self._setup_timers()

        # Start safety monitor
        self._safety.start()

        mode_str = "SIMULATION" if self._sim_mode else "HARDWARE"
        self.get_logger().info(
            f"Sabertooth motor controller started in {mode_str} mode. "
            f"Left=ch{self._param_left_ch}, Right=ch{self._param_right_ch}, "
            f"Timeout={self._param_cmd_timeout_ms}ms"
        )

    # =========================================================================
    # Parameter Declaration and Reading
    # =========================================================================

    def _declare_all_parameters(self) -> None:
        """Declare all ROS2 parameters with defaults and descriptors."""
        # Hardware
        self.declare_parameter('hardware.simulation_mode', False)
        self.declare_parameter('hardware.i2c_bus', 1)
        self.declare_parameter('hardware.pca9685_address', 0x40)
        self.declare_parameter('hardware.pwm_frequency', 50)

        # Channel mapping
        self.declare_parameter('channels.left_motor', 0)
        self.declare_parameter('channels.right_motor', 1)

        # Sabertooth PWM
        self.declare_parameter('sabertooth.pulse_neutral_us', 1500)
        self.declare_parameter('sabertooth.pulse_min_us', 1000)
        self.declare_parameter('sabertooth.pulse_max_us', 2000)
        self.declare_parameter('sabertooth.deadband_us', 30)

        # Kinematics
        self.declare_parameter('kinematics.type', 'tank')
        self.declare_parameter('kinematics.wheel_separation_m', 0.45)
        self.declare_parameter('kinematics.wheel_radius_m', 0.075)
        self.declare_parameter('kinematics.max_linear_speed_ms', 1.0)
        self.declare_parameter('kinematics.max_angular_speed_rads', 2.0)
        self.declare_parameter('kinematics.left_motor_inverted', False)
        self.declare_parameter('kinematics.right_motor_inverted', False)

        # Safety
        self.declare_parameter('safety.command_timeout_ms', 500.0)
        self.declare_parameter('safety.heartbeat_timeout_ms', 2000.0)
        self.declare_parameter('safety.max_speed_teleop', 0.6)
        self.declare_parameter('safety.max_speed_autonomous', 0.4)
        self.declare_parameter('safety.max_speed_rc', 1.0)
        self.declare_parameter('safety.accel_limit', 2.0)
        self.declare_parameter('safety.decel_limit', 4.0)
        self.declare_parameter('safety.emergency_decel_limit', 10.0)
        self.declare_parameter('safety.startup_delay_sec', 1.0)

        # RC input
        self.declare_parameter('rc_input.enabled', True)
        self.declare_parameter('rc_input.ch1_gpio_pin', 18)
        self.declare_parameter('rc_input.ch2_gpio_pin', 22)
        self.declare_parameter('rc_input.ch3_gpio_pin', 13)
        self.declare_parameter('rc_input.pin_numbering', 'BOARD')
        self.declare_parameter('rc_input.pulse_min_us', 1000.0)
        self.declare_parameter('rc_input.pulse_max_us', 2000.0)
        self.declare_parameter('rc_input.pulse_neutral_us', 1500.0)
        self.declare_parameter('rc_input.rc_deadband_us', 50.0)
        self.declare_parameter('rc_input.signal_timeout_ms', 500.0)
        self.declare_parameter('rc_input.estop_threshold_us', 1200.0)
        self.declare_parameter('rc_input.rc_mixing_mode', 'tank')

        # Diagnostics
        self.declare_parameter('diagnostics.status_publish_rate_hz', 10.0)

    def _read_parameters(self) -> None:
        """Read all declared parameters into instance variables."""
        # Hardware
        self._param_sim_mode = self.get_parameter('hardware.simulation_mode').value
        self._param_i2c_bus = self.get_parameter('hardware.i2c_bus').value
        self._param_pca_addr = self.get_parameter('hardware.pca9685_address').value
        self._param_pwm_freq = self.get_parameter('hardware.pwm_frequency').value

        # Channels
        self._param_left_ch = self.get_parameter('channels.left_motor').value
        self._param_right_ch = self.get_parameter('channels.right_motor').value

        # Sabertooth
        self._param_pulse_neutral = self.get_parameter('sabertooth.pulse_neutral_us').value
        self._param_pulse_min = self.get_parameter('sabertooth.pulse_min_us').value
        self._param_pulse_max = self.get_parameter('sabertooth.pulse_max_us').value
        self._param_deadband = self.get_parameter('sabertooth.deadband_us').value

        # Kinematics
        self._param_kinematics_type = self.get_parameter('kinematics.type').value
        self._param_wheel_sep = self.get_parameter('kinematics.wheel_separation_m').value
        self._param_wheel_radius = self.get_parameter('kinematics.wheel_radius_m').value
        self._param_max_linear = self.get_parameter('kinematics.max_linear_speed_ms').value
        self._param_max_angular = self.get_parameter('kinematics.max_angular_speed_rads').value
        self._param_left_inv = self.get_parameter('kinematics.left_motor_inverted').value
        self._param_right_inv = self.get_parameter('kinematics.right_motor_inverted').value

        # Safety
        self._param_cmd_timeout_ms = self.get_parameter('safety.command_timeout_ms').value
        self._param_heartbeat_timeout_ms = self.get_parameter('safety.heartbeat_timeout_ms').value
        self._param_max_speed_teleop = self.get_parameter('safety.max_speed_teleop').value
        self._param_max_speed_auto = self.get_parameter('safety.max_speed_autonomous').value
        self._param_max_speed_rc = self.get_parameter('safety.max_speed_rc').value
        self._param_accel_limit = self.get_parameter('safety.accel_limit').value
        self._param_decel_limit = self.get_parameter('safety.decel_limit').value
        self._param_emergency_decel = self.get_parameter('safety.emergency_decel_limit').value
        self._param_startup_delay = self.get_parameter('safety.startup_delay_sec').value

        # RC input
        self._param_rc_enabled = self.get_parameter('rc_input.enabled').value
        self._param_rc_ch1_pin = self.get_parameter('rc_input.ch1_gpio_pin').value
        self._param_rc_ch2_pin = self.get_parameter('rc_input.ch2_gpio_pin').value
        self._param_rc_ch3_pin = self.get_parameter('rc_input.ch3_gpio_pin').value
        self._param_rc_pin_mode = self.get_parameter('rc_input.pin_numbering').value
        self._param_rc_pulse_min = self.get_parameter('rc_input.pulse_min_us').value
        self._param_rc_pulse_max = self.get_parameter('rc_input.pulse_max_us').value
        self._param_rc_pulse_neutral = self.get_parameter('rc_input.pulse_neutral_us').value
        self._param_rc_deadband = self.get_parameter('rc_input.rc_deadband_us').value
        self._param_rc_timeout = self.get_parameter('rc_input.signal_timeout_ms').value
        self._param_rc_estop_thresh = self.get_parameter('rc_input.estop_threshold_us').value
        self._param_rc_mixing = self.get_parameter('rc_input.rc_mixing_mode').value

        # Diagnostics
        self._param_status_rate = self.get_parameter('diagnostics.status_publish_rate_hz').value

    # =========================================================================
    # Hardware Initialization
    # =========================================================================

    def _initialize_hardware(self) -> None:
        """Create and initialize all hardware driver instances.

        If simulation_mode is True or hardware is not detected,
        falls back to mock drivers automatically.
        """
        # PCA9685 driver
        if self._sim_mode:
            self.get_logger().info("Using mock PWM driver (simulation mode)")
            pwm_driver = MockPWMDriver(self._param_i2c_bus, self._param_pca_addr)
        else:
            pwm_driver = PCA9685Driver(self._param_i2c_bus, self._param_pca_addr)

        # Sabertooth driver (wraps PWM driver)
        self._sabertooth = SabertoothDriver(
            pwm_driver=pwm_driver,
            left_channel=self._param_left_ch,
            right_channel=self._param_right_ch,
            pulse_neutral_us=self._param_pulse_neutral,
            pulse_min_us=self._param_pulse_min,
            pulse_max_us=self._param_pulse_max,
            deadband_us=self._param_deadband,
            left_inverted=self._param_left_inv,
            right_inverted=self._param_right_inv,
            pwm_frequency=self._param_pwm_freq,
        )

        hw_ready = self._sabertooth.initialize()
        if not hw_ready and not self._sim_mode:
            self.get_logger().warn(
                "PCA9685 hardware not detected - falling back to simulation mode. "
                "Check I2C wiring and run: i2cdetect -y -r 1"
            )
            self._sim_mode = True
            # Replace with mock driver since real one failed to initialize
            mock_pwm = MockPWMDriver(self._param_i2c_bus, self._param_pca_addr)
            self._sabertooth = SabertoothDriver(
                pwm_driver=mock_pwm,
                left_channel=self._param_left_ch,
                right_channel=self._param_right_ch,
                pulse_neutral_us=self._param_pulse_neutral,
                pulse_min_us=self._param_pulse_min,
                pulse_max_us=self._param_pulse_max,
                deadband_us=self._param_deadband,
                left_inverted=self._param_left_inv,
                right_inverted=self._param_right_inv,
                pwm_frequency=self._param_pwm_freq,
            )
            self._sabertooth.initialize()

        # RC input
        if self._param_rc_enabled:
            if self._sim_mode:
                self.get_logger().info("Using mock RC input (simulation mode)")
                self._rc_input = MockRCInput(
                    pulse_min_us=self._param_rc_pulse_min,
                    pulse_max_us=self._param_rc_pulse_max,
                    pulse_neutral_us=self._param_rc_pulse_neutral,
                    deadband_us=self._param_rc_deadband,
                    estop_threshold_us=self._param_rc_estop_thresh,
                    rc_mixing_mode=self._param_rc_mixing,
                )
            else:
                self._rc_input = RCInput(
                    ch1_pin=self._param_rc_ch1_pin,
                    ch2_pin=self._param_rc_ch2_pin,
                    ch3_pin=self._param_rc_ch3_pin,
                    pin_numbering=self._param_rc_pin_mode,
                    pulse_min_us=self._param_rc_pulse_min,
                    pulse_max_us=self._param_rc_pulse_max,
                    pulse_neutral_us=self._param_rc_pulse_neutral,
                    deadband_us=self._param_rc_deadband,
                    signal_timeout_ms=self._param_rc_timeout,
                    estop_threshold_us=self._param_rc_estop_thresh,
                    rc_mixing_mode=self._param_rc_mixing,
                )
            self._rc_input.start()
        else:
            self._rc_input = None
            self.get_logger().info("RC input disabled by configuration")

    def _is_jetson_platform(self) -> bool:
        """Detect if running on a Jetson platform."""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().lower()
                return 'jetson' in model or 'nvidia' in model
        except (FileNotFoundError, PermissionError):
            return False

    # =========================================================================
    # ROS2 Subscribers
    # =========================================================================

    def _setup_subscribers(self) -> None:
        """Create ROS2 topic subscriptions."""
        # QoS profile for real-time motor control - best effort, keep latest only
        motor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # /cmd_vel - standard navigation interface
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            motor_qos,
        )

        # /motor_cmd - direct motor control (custom message)
        # We import the message type dynamically to handle the case where
        # the msgs package hasn't been built yet (during development)
        try:
            from sabertooth_motor_controller_msgs.msg import MotorCommand
            self._motor_cmd_sub = self.create_subscription(
                MotorCommand,
                'motor_cmd',
                self._motor_cmd_callback,
                motor_qos,
            )
            self._motor_cmd_available = True
        except ImportError:
            self.get_logger().warn(
                "sabertooth_motor_controller_msgs not found. "
                "/motor_cmd topic disabled. Build the msgs package first: "
                "colcon build --packages-select sabertooth_motor_controller_msgs"
            )
            self._motor_cmd_available = False

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle /cmd_vel messages from teleop or nav stack.

        Converts Twist to motor speeds via kinematics and stores
        as pending command for the next control loop cycle.

        The source is set to TELEOP. If you need autonomous source,
        use the /motor_cmd topic with source=SOURCE_AUTONOMOUS.
        """
        left, right = self._kinematics.twist_to_motor_speeds(
            msg.linear.x, msg.angular.z
        )
        self._pending_left = left
        self._pending_right = right
        self._pending_source = CommandSource.TELEOP
        self._safety.command_received(CommandSource.TELEOP)

    def _motor_cmd_callback(self, msg) -> None:
        """Handle /motor_cmd (MotorCommand) messages for direct motor control.

        Bypasses kinematics - uses left_speed and right_speed directly.
        """
        self._pending_left = float(msg.left_speed)
        self._pending_right = float(msg.right_speed)

        # Map source constant to CommandSource enum
        source_map = {0: CommandSource.AUTONOMOUS, 1: CommandSource.TELEOP,
                      2: CommandSource.RC, 3: CommandSource.ESTOP}
        self._pending_source = source_map.get(msg.source, CommandSource.TELEOP)
        self._safety.command_received(self._pending_source)

    # =========================================================================
    # ROS2 Publishers
    # =========================================================================

    def _setup_publishers(self) -> None:
        """Create ROS2 topic publishers."""
        try:
            from sabertooth_motor_controller_msgs.msg import MotorStatus
            self._status_pub = self.create_publisher(MotorStatus, 'motor_status', 10)
            self._status_msg_available = True
        except ImportError:
            self.get_logger().warn(
                "MotorStatus message not available. /motor_status topic disabled."
            )
            self._status_msg_available = False

    # =========================================================================
    # ROS2 Services
    # =========================================================================

    def _setup_services(self) -> None:
        """Create ROS2 services."""
        # /estop - engage/release software e-stop
        self._estop_srv = self.create_service(
            SetBool, 'estop', self._estop_service_callback
        )

        # /reset - reset from ESTOP or ERROR state
        self._reset_srv = self.create_service(
            Trigger, 'reset', self._reset_service_callback
        )

    def _estop_service_callback(self, request, response):
        """Handle /estop service: engage (True) or release (False) e-stop.

        Engage: Immediately stops motors and enters ESTOP state.
        Release: Marks e-stop as released, but does NOT leave ESTOP state.
                 Must also call /reset service to return to NORMAL.
        """
        self._software_estop = request.data
        self._safety.set_estop(request.data)

        if request.data:
            response.success = True
            response.message = "E-STOP ENGAGED. Motors stopped. Call /reset to resume."
            self.get_logger().warn("E-STOP engaged via service call")
        else:
            response.success = True
            response.message = "E-stop released. Call /reset service to resume normal operation."
            self.get_logger().info("E-stop released via service call")

        return response

    def _reset_service_callback(self, request, response):
        """Handle /reset service: attempt to reset from ESTOP or ERROR.

        Prerequisites:
          - E-stop must be released first
          - Hardware must be healthy (for ERROR state)
        """
        current_state = self._safety.state

        if current_state == SafetyState.ESTOP:
            if self._safety.reset_from_estop():
                response.success = True
                response.message = "Reset successful. Returning to NORMAL operation."
                self.get_logger().info("Reset from ESTOP via service call")
            else:
                response.success = False
                response.message = "Reset failed. Ensure e-stop is released first."
        elif current_state == SafetyState.ERROR:
            if self._safety.reset_from_error():
                response.success = True
                response.message = "Reset from ERROR successful."
                self.get_logger().info("Reset from ERROR via service call")
            else:
                response.success = False
                response.message = "Reset from ERROR failed."
        else:
            response.success = True
            response.message = f"No reset needed. Current state: {current_state.name}"

        return response

    # =========================================================================
    # Timers
    # =========================================================================

    def _setup_timers(self) -> None:
        """Create ROS2 timers for control loop and status publishing."""
        # Main control loop at 50Hz
        control_period = 1.0 / self.CONTROL_LOOP_HZ
        self._control_timer = self.create_timer(
            control_period, self._control_loop_callback
        )

        # Status publishing at configured rate (default 10Hz)
        if self._param_status_rate > 0:
            status_period = 1.0 / self._param_status_rate
            self._status_timer = self.create_timer(
                status_period, self._publish_status
            )

    # =========================================================================
    # Main Control Loop (50Hz)
    # =========================================================================

    def _control_loop_callback(self) -> None:
        """Main control loop, called at 50Hz.

        This is where all the pieces come together:
          1. Compute time delta since last cycle
          2. Read RC input state (if enabled)
          3. Determine active command source and target speeds
          4. Run safety processing (timeouts, speed limits, accel limits)
          5. Output processed speeds to Sabertooth driver
          6. Store state for status publishing

        CRITICAL: The safety monitor's process_speeds() enforces the invariant
        that motors are neutral in ESTOP/TIMEOUT/INITIALIZING/ERROR states.
        We also explicitly call set_neutral() as a defense-in-depth measure.
        """
        # 1. Compute dt
        now = time.monotonic()
        dt = now - self._last_loop_time
        self._last_loop_time = now

        # Clamp dt to prevent huge jumps after system sleep/pause
        dt = min(dt, 0.1)

        # 2. Read RC input
        target_left = self._pending_left
        target_right = self._pending_right
        active_source = self._pending_source

        if self._rc_input is not None:
            rc_state = self._rc_input.get_state()

            # E-stop from RC channel 3 has absolute priority
            if rc_state.estop_active:
                self._safety.set_estop(True)
                target_left, target_right = 0.0, 0.0
                active_source = CommandSource.ESTOP

            # RC override: RC signals present and not in e-stop
            elif rc_state.connected:
                self._safety.set_rc_override(True)
                rc_speeds = self._rc_input.get_motor_speeds()
                if rc_speeds is not None:
                    target_left, target_right = rc_speeds
                else:
                    target_left, target_right = 0.0, 0.0
                active_source = CommandSource.RC
                self._safety.command_received(CommandSource.RC)

            # No RC signals
            else:
                self._safety.set_rc_override(False)
                # Also release RC-triggered e-stop when RC signal is lost
                # (software e-stop via service is separate)
                if not self._software_estop:
                    self._safety.set_estop(False)
        else:
            # Check software e-stop
            if self._software_estop:
                self._safety.set_estop(True)

        # 3. Check timeouts
        self._safety.check_timeouts()

        # 4. Process through safety (speed limit, accel limit, state enforcement)
        processed_left, processed_right, was_limited = self._safety.process_speeds(
            target_left, target_right, active_source, dt
        )

        # 5. Output to hardware
        safety_state = self._safety.state
        if safety_state in (SafetyState.NORMAL, SafetyState.RC_OVERRIDE):
            left_duty, right_duty = self._sabertooth.set_motors(
                processed_left, processed_right
            )
        else:
            # Defense in depth: force neutral in any non-operational state
            self._sabertooth.set_neutral()
            left_duty = right_duty = self._sabertooth.get_neutral_duty_cycle()

        # 6. Store state for status publishing
        self._last_left_duty = left_duty
        self._last_right_duty = right_duty
        self._last_was_limited = was_limited

    # =========================================================================
    # Status Publishing
    # =========================================================================

    def _publish_status(self) -> None:
        """Build and publish MotorStatus message with current state."""
        if not self._status_msg_available:
            return

        try:
            from sabertooth_motor_controller_msgs.msg import MotorStatus
        except ImportError:
            return

        msg = MotorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Motor speeds
        msg.left_speed_commanded = float(self._pending_left)
        msg.right_speed_commanded = float(self._pending_right)

        current_left, current_right = self._safety.current_speeds
        msg.left_speed_actual = float(current_left)
        msg.right_speed_actual = float(current_right)

        msg.left_pwm_duty_cycle = self._last_left_duty
        msg.right_pwm_duty_cycle = self._last_right_duty

        # Safety state
        state_map = {
            SafetyState.NORMAL: MotorStatus.STATE_NORMAL,
            SafetyState.TIMEOUT: MotorStatus.STATE_TIMEOUT,
            SafetyState.ESTOP: MotorStatus.STATE_ESTOP,
            SafetyState.RC_OVERRIDE: MotorStatus.STATE_RC_OVERRIDE,
            SafetyState.INITIALIZING: MotorStatus.STATE_INITIALIZING,
            SafetyState.ERROR: MotorStatus.STATE_ERROR,
        }
        msg.safety_state = state_map.get(self._safety.state, MotorStatus.STATE_ERROR)

        # Active source
        source_map = {
            CommandSource.AUTONOMOUS: 0,
            CommandSource.TELEOP: 1,
            CommandSource.RC: 2,
            CommandSource.ESTOP: 3,
        }
        msg.active_source = source_map.get(self._safety.get_active_source(), 0)

        # Flags
        msg.hardware_present = not self._sim_mode
        msg.estop_active = self._safety.state == SafetyState.ESTOP
        msg.command_timeout = self._safety.state == SafetyState.TIMEOUT
        msg.speed_limited = self._last_was_limited

        # RC status
        if self._rc_input is not None:
            rc_state = self._rc_input.get_state()
            msg.rc_connected = rc_state.connected
            msg.rc_ch1_pulse_us = float(rc_state.ch1.pulse_us)
            msg.rc_ch2_pulse_us = float(rc_state.ch2.pulse_us)
            msg.rc_ch3_pulse_us = float(rc_state.ch3.pulse_us)
        else:
            msg.rc_connected = False

        # Timing
        msg.last_command_age_sec = self._safety.get_last_command_age()
        msg.uptime_sec = self._safety.get_uptime()

        self._status_pub.publish(msg)

    # =========================================================================
    # State Change Callback
    # =========================================================================

    def _on_safety_state_change(self, old_state: SafetyState, new_state: SafetyState) -> None:
        """Called by safety monitor when state changes. Logs the transition."""
        if new_state == SafetyState.ESTOP:
            self.get_logger().error(f"SAFETY: {old_state.name} -> E-STOP. Motors stopped.")
        elif new_state == SafetyState.TIMEOUT:
            self.get_logger().warn(f"SAFETY: {old_state.name} -> TIMEOUT. No commands received.")
        elif new_state == SafetyState.ERROR:
            self.get_logger().error(f"SAFETY: {old_state.name} -> ERROR. Hardware fault.")
        elif new_state == SafetyState.NORMAL:
            self.get_logger().info(f"SAFETY: {old_state.name} -> NORMAL. Operating normally.")
        elif new_state == SafetyState.RC_OVERRIDE:
            self.get_logger().info(f"SAFETY: {old_state.name} -> RC_OVERRIDE. RC has control.")

    # =========================================================================
    # Shutdown
    # =========================================================================

    def destroy_node(self) -> None:
        """Clean shutdown: set motors to neutral, stop RC, cleanup.

        CRITICAL: Motors MUST return to neutral before we exit.
        The PCA9685 holds the last PWM value if not explicitly set.
        """
        self.get_logger().info("Shutting down motor controller...")

        # Stop RC input
        if self._rc_input is not None:
            self._rc_input.stop()

        # Shutdown Sabertooth (sets neutral internally)
        self._sabertooth.shutdown()

        self.get_logger().info("Motor controller shutdown complete. Motors at neutral.")
        super().destroy_node()


def main(args=None):
    """Entry point for 'ros2 run sabertooth_motor_controller motor_controller_node'."""
    rclpy.init(args=args)
    node = SabertoothMotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
