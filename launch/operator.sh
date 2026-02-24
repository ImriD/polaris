#!/bin/bash
# Reconnect the operator TUI to an already-running rig.
# Use launch/sim.sh for the full setup. This is for reconnecting.
#
# Usage:
#   ./launch/operator.sh

set -euo pipefail

echo "Starting tui_dashboard on fastgpu..."
echo "  Tab = cycle views | WASD = drive | G = gear | Space = stop | Q = quit"
echo ""
exec ssh -t f 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH && ~/src/experimental/polaris/target/release/tui_dashboard'
