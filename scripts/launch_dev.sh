#!/bin/bash

# Build (optional) and launch the dev vehicle_node, or run one sequence.
# Usage: bash launch_dev.sh [--target N] [--build]
#
#   (no target)  vehicle_node - the adapter published as topics
#   --target 1   takeoff      GUIDED, arm, climb to 5 m
#   --target 2   goto_local   fly to one ENU waypoint
#   --target 3   land         LAND mode, wait for disarm
#
# Two terminals: start the node with no target, then run sequences against it
# as many times as you like. Sequences are standalone - each owns its node,
# runs start to finish, and confirms what happened by watching telemetry.
#
# Assumes ArduPilot SITL and MAVROS are already up - this script starts
# neither, same as launch_fs.sh.
#
# Flags can come in either order.

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_dev"

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
    1) EXECUTABLE="takeoff" ;;
    2) EXECUTABLE="goto_local" ;;
    3) EXECUTABLE="land" ;;
    *) echo "❌ Unknown target: $TARGET (expected 1-3, or none)"; exit 1 ;;
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
    echo "🚀 Launching vehicle_node (SITL)..."
    ros2 launch "$PACKAGE_NAME" sitl.launch.py
else
    echo "🚀 Running $EXECUTABLE (target $TARGET)..."
    ros2 run "$PACKAGE_NAME" "$EXECUTABLE"
fi
