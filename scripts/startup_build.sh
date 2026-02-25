#!/bin/bash
# =============================================================================
# Build script for the motor controller workspace
#
# Builds ROS2 packages and sources the workspace. Run this after
# pulling new code or making changes.
#
# Usage:
#   ./startup_build.sh                   # Build all packages
#   ./startup_build.sh --controller      # Build motor controller only
#   ./startup_build.sh --msgs            # Build messages only
#   ./startup_build.sh --clean           # Clean build (delete build/install/log)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

BUILD_PACKAGES=""
CLEAN=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --controller)
            BUILD_PACKAGES="--packages-select sabertooth_motor_controller"
            shift
            ;;
        --msgs)
            BUILD_PACKAGES="--packages-select sabertooth_motor_controller_msgs"
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --help|-h)
            echo "Usage: $(basename "$0") [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --controller   Build motor controller package only"
            echo "  --msgs         Build messages package only"
            echo "  --clean        Clean build directories before building"
            echo "  -h, --help     Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

cd "$WS_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

if [ "$CLEAN" = true ]; then
    echo "Cleaning build directories..."
    rm -rf build/ install/ log/
    echo "Clean complete."
    echo ""
fi

echo "=========================================="
echo "  Building Motor Controller Workspace"
echo "=========================================="
echo "  Directory: $WS_DIR"
if [ -n "$BUILD_PACKAGES" ]; then
    echo "  Packages:  $(echo $BUILD_PACKAGES | sed 's/--packages-select //')"
else
    echo "  Packages:  ALL"
fi
echo "=========================================="
echo ""

colcon build $BUILD_PACKAGES

echo ""
echo "Build complete. Source the workspace with:"
echo "  source $WS_DIR/install/setup.bash"
