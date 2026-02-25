#!/bin/bash
# =============================================================================
# Startup script for Sabertooth Motor Controller Node
#
# Starts the motor controller with PCA9685 hardware (or simulation mode).
# Auto-detects Jetson platform and I2C bus (prefers bus 7 on Orin Nano).
#
# Usage:
#   ./startup_motor_controller.sh                  # Default (auto-detect)
#   ./startup_motor_controller.sh --sim            # Force simulation mode
#   ./startup_motor_controller.sh --debug          # Enable debug logging
#   ./startup_motor_controller.sh --config FILE    # Custom config file
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

# Build launch command
CMD="ros2 launch sabertooth_motor_controller motor_controller.launch.py"
CMD="$CMD simulation_mode:=$SIM_MODE"
CMD="$CMD log_level:=$LOG_LEVEL"

if [ -n "$CONFIG_FILE" ]; then
    CMD="$CMD config_file:=$CONFIG_FILE"
fi

echo "=========================================="
echo "  Sabertooth Motor Controller"
echo "=========================================="
echo "  Simulation: $SIM_MODE"
echo "  Log level:  $LOG_LEVEL"
if [ -n "$CONFIG_FILE" ]; then
    echo "  Config:     $CONFIG_FILE"
fi
echo "=========================================="
echo ""

exec $CMD
