#!/bin/bash

# Build (optional) and launch the sequence runner, or one test script.
# Usage: bash launch_seqs.sh [--target N] [--build]
#
#   (no target)  the sequence runner - sequences are requested by udl_aa_fc
#   --target 1   test_goto_local   ENU diamond around the origin
#   --target 2   test_goto_body    RFU cross with a spin at each end
#   --target 3   test_velocity     ENU diamond flown on timed velocity legs
#   --target 4   test_goto_global  WGS-84 waypoints from GCS clicks
#   --target 5   aruco_lander      scan for the marker, settle, servo down
#
# The test scripts are standalone: each owns its node, runs start to finish, and
# confirms nothing. They bypass the runner entirely - they are not sequences and
# it knows nothing about them. Target 5 is the exception to "runs start to
# finish": it is closed loop on the detector and ends when it hands off to LAND,
# or on ctrl-C.
#
# Start the runner before udl_aa_fc. The commander's `live` gate is a subscriber
# count on the request topic, so with this down it publishes nothing and
# silently looks idle rather than failing.
#
# Flags can come in either order.

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_seqs"

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
    0) EXECUTABLE="$PACKAGE_NAME" ;;
    1) EXECUTABLE="test_goto_local" ;;
    2) EXECUTABLE="test_goto_body" ;;
    3) EXECUTABLE="test_velocity" ;;
    4) EXECUTABLE="test_goto_global" ;;
    5) EXECUTABLE="aruco_lander" ;;
    *) echo "❌ Unknown target: $TARGET (expected 1-5, or none)"; exit 1 ;;
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

if [[ "$TARGET" == "0" ]]; then
    echo "🚀 Launching sequence runner..."
else
    echo "🚀 Running $EXECUTABLE (target $TARGET)..."
fi
ros2 run "$PACKAGE_NAME" "$EXECUTABLE"
