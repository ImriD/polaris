#!/bin/bash
# Sync polaris workspace to fastgpu. Run with --build to compile on remote after sync.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REMOTE="f:~/src/experimental/polaris/"

ssh f "mkdir -p ~/src/experimental/polaris"
scp -r \
  "$SCRIPT_DIR/Cargo.toml" "$SCRIPT_DIR/Cargo.lock" \
  "$SCRIPT_DIR/polaris_bridge" "$SCRIPT_DIR/polaris_control" \
  "$SCRIPT_DIR/polaris_operator" "$SCRIPT_DIR/polaris_sim" \
  "$SCRIPT_DIR/polaris_wheel" "$SCRIPT_DIR/polaris_msgs" \
  "$SCRIPT_DIR/launch" \
  "$REMOTE"

echo "synced → $REMOTE"

if [ "${1:-}" = "--build" ]; then
  echo "building on remote..."
  ssh f "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && cd ~/src/experimental/polaris && cargo build --release -p polaris_bridge -p polaris_operator -p polaris_sim 2>&1 | tail -5"
fi
