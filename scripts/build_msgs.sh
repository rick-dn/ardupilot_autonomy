#!/bin/bash

# Build udl_aa_msgs. Nothing to launch - the package is interface definitions
# only and produces no executable.
# Usage: bash build_msgs.sh
#
# --symlink-install is deliberately not used here. It links source files into
# install/, which does nothing for a package whose Python and C++ are generated
# at build time, and leaves stale generated code behind when a .msg changes.
#
# Always rebuilt from clean for the same reason: a removed or renamed field
# leaves its generated module in place otherwise, and dependents keep importing
# something the .msg no longer declares.

set -e

WORKSPACE="/data/projects/ardupilot_autonomy/ardu_ws"
PACKAGE_NAME="udl_aa_msgs"

echo "📂 Changing to workspace..."
cd "$WORKSPACE" || { echo "❌ Failed to cd to $WORKSPACE"; exit 1; }

echo "🧹 Cleaning $PACKAGE_NAME artifacts..."
rm -rf build/"$PACKAGE_NAME" install/"$PACKAGE_NAME"

echo "🔨 Building $PACKAGE_NAME..."
colcon build --packages-select "$PACKAGE_NAME"

echo "✅ $PACKAGE_NAME built."
echo "⚠️  Dependents link against the generated interfaces - rebuild udl_aa_fs"
echo "   and udl_aa_fc after any .msg change, in a freshly sourced shell."
