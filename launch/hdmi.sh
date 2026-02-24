#!/bin/bash
# Open tui_dashboard in monitor mode on fastgpu's physical HDMI display via alacritty on Wayland.
set -euo pipefail

ssh f "WAYLAND_DISPLAY=wayland-1 XDG_RUNTIME_DIR=/run/user/1000 \
  alacritty -e bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH; ~/src/experimental/polaris/target/release/tui_dashboard --monitor; echo EXITED; read'"
