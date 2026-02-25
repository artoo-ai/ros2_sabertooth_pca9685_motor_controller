#!/bin/bash
# =============================================================================
# Startup script for Motor Status Monitor
#
# Subscribes to /motor_status and displays real-time motor state.
# Useful for debugging - run alongside any other startup script
# to see what the motor controller is doing.
#
# Usage:
#   ./startup_monitor.sh                 # Default (1Hz echo)
#   ./startup_monitor.sh --hz            # Show publish rate only
#   ./startup_monitor.sh --raw           # Raw message dump
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

MODE="echo"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --hz)
            MODE="hz"
            shift
            ;;
        --raw)
            MODE="raw"
            shift
            ;;
        --help|-h)
            echo "Usage: $(basename "$0") [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --hz     Show publish rate instead of data"
            echo "  --raw    Show raw message data (high rate)"
            echo "  -h       Show this help message"
            echo ""
            echo "Displays:"
            echo "  - Safety state (NORMAL, TIMEOUT, ESTOP, etc.)"
            echo "  - Commanded vs actual motor speeds"
            echo "  - PWM duty cycles"
            echo "  - Active command source"
            echo "  - RC input status"
            echo "  - E-stop and timeout flags"
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

echo "=========================================="
echo "  Motor Status Monitor"
echo "=========================================="
echo "  Topic: /motor_status"
echo "  Mode:  $MODE"
echo "  Press Ctrl+C to stop"
echo "=========================================="
echo ""

case $MODE in
    echo)
        exec ros2 topic echo /motor_status
        ;;
    hz)
        exec ros2 topic hz /motor_status
        ;;
    raw)
        exec ros2 topic echo /motor_status --no-arr
        ;;
esac
