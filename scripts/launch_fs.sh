#!/bin/bash

# Build (optional) and launch vehicle_controller
# Usage: bash run_vehicle_controller.sh [--build]

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_fs"

echo "📂 Changing to workspace..."
cd "$WORKSPACE" || { echo "❌ Failed to cd to $WORKSPACE"; exit 1; }

if [[ "$1" == "--build" ]]; then
    echo "🧹 Cleaning $PACKAGE_NAME artifacts..."
    rm -rf build/"$PACKAGE_NAME" install/"$PACKAGE_NAME" log/

    echo "🔨 Building with symlink..."
    colcon build --packages-select "$PACKAGE_NAME" --symlink-install
fi

echo "📦 Sourcing workspace..."
source install/setup.bash

echo "🚀 Launching vehicle_controller..."
ros2 launch "$PACKAGE_NAME" vehicle_controller.launch.py
