#!/bin/bash

# Build (optional) and launch the physical-flight stack: vehicle_node plus
# rc_monitor.
# Usage: bash launch_flight.sh [--build]
#
# Sequences are triggered by RC switch, not from here:
#   channels[7] > 1800  ->  takeoff
#   channels[5] > 1800  ->  land
#
# rc_monitor is the version carried over from the Pi unchanged. It triggers on
# a rising edge only - flicking a switch back down does nothing, and a switch
# already high at startup fires immediately. See rc_monitor.py for the full
# list of known quirks.
#
# launch_dev.sh --target N still works against this node for ground testing;
# the topics are identical.
#
# Assumes MAVROS is already up - on the Pi that is mavros.service.

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_dev"

BUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build) BUILD=1; shift ;;
        *) echo "❌ Unknown option: $1"; exit 1 ;;
    esac
done

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

echo "🚀 Launching vehicle_node + rc_monitor (physical flight)..."
ros2 launch "$PACKAGE_NAME" flight.launch.py
