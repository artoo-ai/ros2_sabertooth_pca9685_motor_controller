#!/bin/bash
# =============================================================================
# Startup script for Web GUI
#
# Starts the motor controller AND web-based GUI for browser control.
# Open http://localhost:5000 (or your Jetson's IP) in any browser.
#
# Features:
#   - WASD / Arrow keys for driving
#   - On-screen touch buttons for mobile/tablet
#   - E-Stop button (always visible, top-right)
#   - Speed slider
#   - Real-time motor status from any command source
#
# Requires: flask
#   pip install flask
#
# Usage:
#   ./startup_gui.sh                     # Default (port 5000)
#   ./startup_gui.sh --sim               # Force simulation mode
#   ./startup_gui.sh --port 8080         # Custom port
#   ./startup_gui.sh --debug             # Enable debug logging
#   ./startup_gui.sh --config FILE       # Custom config file
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Defaults
SIM_MODE="false"
LOG_LEVEL="info"
GUI_PORT="5000"
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
        --port)
            GUI_PORT="$2"
            shift 2
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
            echo "  --port PORT           Web GUI port (default: 5000)"
            echo "  --debug               Enable debug logging"
            echo "  --config FILE         Path to custom parameter YAML file"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "Keyboard Controls (in browser):"
            echo "  W / Up Arrow     Forward"
            echo "  S / Down Arrow   Backward"
            echo "  A / Left Arrow   Turn left"
            echo "  D / Right Arrow  Turn right"
            echo "  Space            E-Stop toggle"
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

# Check flask is installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo "WARNING: Flask not installed."
    echo "  Install with: pip install flask"
    echo ""
fi

# Build launch command
CMD="ros2 launch sabertooth_motor_controller gui.launch.py"
CMD="$CMD simulation_mode:=$SIM_MODE"
CMD="$CMD gui_port:=$GUI_PORT"

if [ -n "$CONFIG_FILE" ]; then
    CMD="$CMD config_file:=$CONFIG_FILE"
fi

# Get IP address for display
IP_ADDR=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")

echo "=========================================="
echo "  Sabertooth Web GUI"
echo "=========================================="
echo "  Simulation: $SIM_MODE"
echo "  Port:       $GUI_PORT"
if [ -n "$CONFIG_FILE" ]; then
    echo "  Config:     $CONFIG_FILE"
fi
echo ""
echo "  Local:   http://localhost:$GUI_PORT"
echo "  Network: http://$IP_ADDR:$GUI_PORT"
echo "=========================================="
echo ""

exec $CMD
