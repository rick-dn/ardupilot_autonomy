#!/bin/bash

# Build (optional) and launch the flight commander, or one test script.
# Usage: bash launch_fc.sh [--target N] [--build]
#
#   (no target)  the flight commander - sequences are started from the GUI
#   --target 1   test_goto_local   ENU diamond around the origin
#   --target 2   test_goto_body    RFU cross with a spin at each end
#   --target 3   test_velocity     ENU diamond flown on timed velocity legs
#   --target 4   test_goto_global  WGS-84 waypoints from GCS clicks
#
# The test scripts are standalone: each owns its node, runs start to finish,
# and confirms nothing. They are not sequences and the commander knows nothing
# about them.
#
# Flags can come in either order.

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_fc"

TARGET=0
BUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)  BUILD=1; shift ;;
        --target) TARGET="$2"; shift 2 ;;
        *) echo "❌ Unknown option: $1"; exit 1 ;;
    esac
done

# Entry points as named in setup.py console_scripts.
case "$TARGET" in
    0) EXECUTABLE="" ;;
    1) EXECUTABLE="test_goto_local" ;;
    2) EXECUTABLE="test_goto_body" ;;
    3) EXECUTABLE="test_velocity" ;;
    4) EXECUTABLE="test_goto_global" ;;
    *) echo "❌ Unknown target: $TARGET (expected 1-4, or none)"; exit 1 ;;
esac

echo "📂 Changing to workspace..."
cd "$WORKSPACE" || { echo "❌ Failed to cd to $WORKSPACE"; exit 1; }

if [[ "$BUILD" == "1" ]]; then
    echo "🧹 Cleaning $PACKAGE_NAME artifacts..."
    rm -rf build/"$PACKAGE_NAME" install/"$PACKAGE_NAME"

    echo "🔨 Building with symlink..."
    colcon build --packages-select "$PACKAGE_NAME" --symlink-install
fi

echo "📦 Sourcing workspace..."
source install/setup.bash

if [[ -z "$EXECUTABLE" ]]; then
    echo "🚀 Launching flight commander..."
    ros2 launch "$PACKAGE_NAME" udl_aa_fc.launch.py
else
    echo "🚀 Running $EXECUTABLE (target $TARGET)..."
    ros2 run "$PACKAGE_NAME" "$EXECUTABLE"
fi
