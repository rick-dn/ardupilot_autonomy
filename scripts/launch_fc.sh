#!/bin/bash

# Build (optional) and launch the flight commander.
# Usage: bash launch_fc.sh [--build]
#
# The deliberative layer: it evaluates the safety conditions, polls the
# operator, and publishes a sequence request. It runs no sequences itself, so
# start udl_aa_seqs as well - and start it first. The commander's `live` gate is
# a subscriber count on the request topic, so with the runner down it publishes
# nothing and silently looks idle rather than failing.
#
# The standalone test scripts moved with the sequences. They are
# `bash launch_seqs.sh --target N` now.

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_fc"

echo "📂 Changing to workspace..."
cd "$WORKSPACE" || { echo "❌ Failed to cd to $WORKSPACE"; exit 1; }

if [[ "$1" == "--build" ]]; then
    echo "🧹 Cleaning $PACKAGE_NAME artifacts..."
    rm -rf build/"$PACKAGE_NAME" install/"$PACKAGE_NAME"

    echo "🔨 Building with symlink..."
    colcon build --packages-select "$PACKAGE_NAME" --symlink-install
fi

echo "📦 Sourcing workspace..."
source install/setup.bash

echo "🚀 Launching flight commander..."
ros2 launch "$PACKAGE_NAME" udl_aa_fc.launch.py
