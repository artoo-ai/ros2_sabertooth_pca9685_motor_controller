"""Web GUI node for the Sabertooth motor controller.

Provides a browser-based interface for:
  - Monitoring motor status (subscribes to /motor_status)
  - Keyboard/touch driving (publishes to /cmd_vel)
  - E-stop control (calls /estop service)
  - Reset from fault states (calls /reset service)

This node does NOT control motors directly. It communicates with the
motor controller node entirely through standard ROS2 interfaces.

ARCHITECTURE:
  Thread 1 (daemon): rclpy.spin() - processes ROS2 callbacks
  Thread 2 (main):   Flask server - handles HTTP requests

  Shared state between threads is protected by threading.Lock.

SAFETY:
  The GUI publishes /cmd_vel commands at 10Hz while keys are held.
  When the browser tab closes or keys are released, commands stop.
  The motor controller's 500ms command timeout stops the motors
  automatically if the GUI becomes unresponsive.

USAGE:
  ros2 launch sabertooth_motor_controller gui.launch.py simulation_mode:=true
  # Then open http://localhost:5000 in a browser
"""

import os
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

from flask import Flask, jsonify, request, send_from_directory

# State name mappings (match MotorStatus.msg constants)
SAFETY_STATE_NAMES = {
    0: 'NORMAL',
    1: 'TIMEOUT',
    2: 'ESTOP',
    3: 'RC_OVERRIDE',
    4: 'INITIALIZING',
    5: 'ERROR',
}

SOURCE_NAMES = {
    0: 'AUTONOMOUS',
    1: 'TELEOP',
    2: 'RC',
    3: 'ESTOP',
}


class MotorControllerGUI(Node):
    """ROS2 node that runs a Flask web server for motor controller GUI."""

    def __init__(self):
        super().__init__('motor_controller_gui')

        # Parameters
        self.declare_parameter('port', 5000)
        self._port = self.get_parameter('port').value

        # Thread-safe shared state
        self._lock = threading.Lock()
        self._latest_status = {}
        self._connected = False
        self._last_status_time = 0.0

        # ROS2 setup
        self._setup_ros2()

        # Flask setup
        self._web_dir = os.path.join(os.path.dirname(__file__), 'web')
        self._app = Flask(__name__)
        self._setup_routes()

    def _setup_ros2(self):
        """Create all ROS2 subscriptions, publishers, and service clients."""
        # Try to import custom messages (may not be built yet)
        try:
            from sabertooth_motor_controller_msgs.msg import MotorStatus
            self._status_sub = self.create_subscription(
                MotorStatus, 'motor_status', self._status_callback, 10)
            self._has_status_msg = True
            self.get_logger().info('Subscribed to /motor_status')
        except ImportError:
            self.get_logger().warn(
                'sabertooth_motor_controller_msgs not found. '
                'Status monitoring disabled. Build the msgs package first.')
            self._has_status_msg = False

        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._estop_client = self.create_client(SetBool, 'estop')
        self._reset_client = self.create_client(Trigger, 'reset')

    def _status_callback(self, msg):
        """ROS2 callback: store latest MotorStatus as a JSON-friendly dict."""
        with self._lock:
            self._latest_status = {
                'left_speed_commanded': round(msg.left_speed_commanded, 4),
                'right_speed_commanded': round(msg.right_speed_commanded, 4),
                'left_speed_actual': round(msg.left_speed_actual, 4),
                'right_speed_actual': round(msg.right_speed_actual, 4),
                'left_pwm_duty_cycle': msg.left_pwm_duty_cycle,
                'right_pwm_duty_cycle': msg.right_pwm_duty_cycle,
                'safety_state': msg.safety_state,
                'safety_state_name': SAFETY_STATE_NAMES.get(
                    msg.safety_state, 'UNKNOWN'),
                'active_source': msg.active_source,
                'active_source_name': SOURCE_NAMES.get(
                    msg.active_source, 'UNKNOWN'),
                'hardware_present': msg.hardware_present,
                'rc_connected': msg.rc_connected,
                'estop_active': msg.estop_active,
                'command_timeout': msg.command_timeout,
                'speed_limited': msg.speed_limited,
                'last_command_age_sec': round(msg.last_command_age_sec, 3),
                'uptime_sec': round(msg.uptime_sec, 1),
                'rc_ch1_pulse_us': round(msg.rc_ch1_pulse_us, 1),
                'rc_ch2_pulse_us': round(msg.rc_ch2_pulse_us, 1),
                'rc_ch3_pulse_us': round(msg.rc_ch3_pulse_us, 1),
            }
            self._connected = True
            self._last_status_time = time.time()

    def _setup_routes(self):
        """Register Flask routes."""

        @self._app.route('/')
        def index():
            return send_from_directory(self._web_dir, 'index.html')

        @self._app.route('/api/status')
        def get_status():
            with self._lock:
                status = dict(self._latest_status)
                status['connected'] = self._connected
                # Mark disconnected if no status for >1.5 seconds
                if self._connected and (
                        time.time() - self._last_status_time > 1.5):
                    status['connected'] = False
            return jsonify(status)

        @self._app.route('/api/cmd_vel', methods=['POST'])
        def post_cmd_vel():
            data = request.get_json()
            linear_x = max(-1.0, min(1.0, float(data.get('linear_x', 0))))
            angular_z = max(-1.0, min(1.0, float(data.get('angular_z', 0))))

            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self._cmd_vel_pub.publish(msg)

            return jsonify({'ok': True})

        @self._app.route('/api/estop', methods=['POST'])
        def post_estop():
            data = request.get_json()
            engage = bool(data.get('engage', True))

            if not self._estop_client.service_is_ready():
                return jsonify({
                    'success': False,
                    'message': '/estop service not available'
                }), 503

            req = SetBool.Request()
            req.data = engage
            future = self._estop_client.call_async(req)

            # Wait for service response (rclpy spin thread processes it)
            timeout = 2.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.01)

            if future.done():
                result = future.result()
                return jsonify({
                    'success': result.success,
                    'message': result.message
                })
            return jsonify({
                'success': False,
                'message': 'Service call timed out'
            }), 504

        @self._app.route('/api/reset', methods=['POST'])
        def post_reset():
            if not self._reset_client.service_is_ready():
                return jsonify({
                    'success': False,
                    'message': '/reset service not available'
                }), 503

            req = Trigger.Request()
            future = self._reset_client.call_async(req)

            timeout = 2.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.01)

            if future.done():
                result = future.result()
                return jsonify({
                    'success': result.success,
                    'message': result.message
                })
            return jsonify({
                'success': False,
                'message': 'Service call timed out'
            }), 504

    def run_flask(self):
        """Start the Flask server (blocking)."""
        import logging as _logging
        # Suppress Flask's default request logging
        _logging.getLogger('werkzeug').setLevel(_logging.WARNING)

        self.get_logger().info(
            f'Web GUI available at http://0.0.0.0:{self._port}')
        self._app.run(
            host='0.0.0.0',
            port=self._port,
            debug=False,
            use_reloader=False,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerGUI()

    # Spin ROS2 in a background daemon thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_flask()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
