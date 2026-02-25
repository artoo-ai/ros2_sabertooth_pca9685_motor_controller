#!/bin/bash
# =============================================================================
# Startup script for Teleop (Keyboard) Control
#
# Starts the motor controller AND keyboard teleoperation together.
# Use WASD or IJKL keys to drive the robot.
#
# Requires: ros-humble-teleop-twist-keyboard
#   sudo apt install ros-humble-teleop-twist-keyboard
#
# Usage:
#   ./startup_teleop.sh                  # Default (auto-detect hardware)
#   ./startup_teleop.sh --sim            # Force simulation mode
#   ./startup_teleop.sh --debug          # Enable debug logging
#   ./startup_teleop.sh --config FILE    # Custom config file
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Defaults
SIM_MODE="false"
LOG_LEVEL="info"
CONFIG_FILE=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --sim|--simulation)
            SIM_MODE="true"
            shift
            ;;
        --debug)
            LOG_LEVEL="debug"
            shift
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $(basename "$0") [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --sim, --simulation   Force simulation mode (no hardware)"
            echo "  --debug               Enable debug logging"
            echo "  --config FILE         Path to custom parameter YAML file"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "Keyboard Controls (teleop_twist_keyboard):"
            echo "  u  i  o    Forward-left, Forward, Forward-right"
            echo "  j  k  l    Turn-left, Stop, Turn-right"
            echo "  m  ,  .    Back-left, Backward, Back-right"
            echo ""
            echo "  q/z: increase/decrease max speed by 10%"
            echo "  w/x: increase/decrease linear speed by 10%"
            echo "  e/c: increase/decrease angular speed by 10%"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "ERROR: Workspace not built. Run first:"
    echo "  cd $WS_DIR && colcon build"
    exit 1
fi

# Check teleop_twist_keyboard is installed
if ! ros2 pkg list 2>/dev/null | grep -q teleop_twist_keyboard; then
    echo "WARNING: teleop_twist_keyboard not installed."
    echo "  Install with: sudo apt install ros-humble-teleop-twist-keyboard"
    echo ""
fi

# Build launch command
CMD="ros2 launch sabertooth_motor_controller teleop.launch.py"
CMD="$CMD simulation_mode:=$SIM_MODE"

if [ -n "$CONFIG_FILE" ]; then
    CMD="$CMD config_file:=$CONFIG_FILE"
fi

echo "=========================================="
echo "  Sabertooth Teleop (Keyboard)"
echo "=========================================="
echo "  Simulation: $SIM_MODE"
if [ -n "$CONFIG_FILE" ]; then
    echo "  Config:     $CONFIG_FILE"
fi
echo ""
echo "  Controls: i=fwd, ,=back, j=left, l=right, k=stop"
echo "=========================================="
echo ""

exec $CMD
