#!/bin/bash
# Launch the full simulation: SITL + nodes + G923 wheel.
# One command. Cleans up everything (local + remote) before starting fresh.
#
# Usage:
#   ./launch/sim.sh              # full launch
#   ./launch/sim.sh --skip-sitl  # SITL already running, just reconnect wheel

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"
SKIP_SITL=false

for arg in "$@"; do
  case "$arg" in
    --skip-sitl) SKIP_SITL=true ;;
  esac
done

# --- Clean up local processes ---
echo "Cleaning up local..."
pkill -f polaris_wheel 2>/dev/null || true
sleep 1

# --- Remote: launch SITL + vehicle nodes on fastgpu ---
if [ "$SKIP_SITL" = false ]; then
  echo "Syncing code to fastgpu..."
  bash "${WORKSPACE}/sync.sh" --build

  echo "Launching SITL + vehicle stack on fastgpu..."
  ssh f 'bash -s' <<'REMOTE'
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/mavproxy-venv/bin/activate
    export PATH="$HOME/.local/bin:$HOME/ardupilot/Tools/autotest:$PATH"
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH
    export RUST_LOG=info
    BIN=~/src/experimental/polaris/target/release

    # Kill everything
    pkill -9 -f "ardurover|mavproxy|sim_vehicle|pixhawk_bridge|vehicle_control|wheel_bridge|tui_dashboard" 2>/dev/null
    true
    sleep 3

    # PTY wrapper — mavproxy needs a terminal to function
    cat > /tmp/run_pty.py << 'PYEOF'
import pty, os, sys
logfile = sys.argv[1]
cmd = sys.argv[2:]
log = open(logfile, "w")
def read(fd):
    data = os.read(fd, 4096)
    log.write(data.decode(errors="replace"))
    log.flush()
    return data
pty.spawn(cmd, read)
PYEOF

    # sim_vehicle.py handles ardurover + mavproxy + clock (Python 3.12 venv)
    cd ~/ardupilot
    nohup python3 /tmp/run_pty.py /tmp/sim_vehicle.log \
      python3 Tools/autotest/sim_vehicle.py -v Rover --speedup 1 -I0 --no-rebuild \
      --add-param-file Tools/autotest/default_params/rover.parm -D \
      < /dev/null > /dev/null 2>&1 &
    echo "sim_vehicle started"
    sleep 20

    # Vehicle nodes
    PIXHAWK_ADDR=tcpout:127.0.0.1:5762 \
      nohup $BIN/pixhawk_bridge_node > /tmp/pixhawk_bridge.log 2>&1 &
    sleep 3

    SPEED_LIMIT=8 \
      nohup $BIN/vehicle_control_node > /tmp/vehicle_control.log 2>&1 &
    sleep 3

    nohup $BIN/wheel_bridge_node > /tmp/wheel_bridge.log 2>&1 &
    sleep 1

    # Verify
    STATE=$(timeout 5 ros2 topic echo /vehicle/state --once 2>/dev/null | head -1)
    if [ -n "$STATE" ]; then
      echo "OK: vehicle state publishing"
    else
      echo "WARN: no vehicle state yet (bridge may still be connecting)"
    fi
REMOTE
else
  # Sync latest code, then restart wheel_bridge
  echo "Syncing code to fastgpu..."
  bash "${WORKSPACE}/sync.sh"

  echo "Restarting wheel_bridge on fastgpu..."
  ssh f 'bash -s' <<'REMOTE'
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH
    export RUST_LOG=info
    BIN=~/src/experimental/polaris/target/release

    pkill -9 -f wheel_bridge 2>/dev/null
    true
    sleep 1

    pushd ~/src/experimental/polaris
    cargo build --bin wheel_bridge_node --release
    popd
    nohup $BIN/wheel_bridge_node > /tmp/wheel_bridge.log 2>&1 &
    sleep 1
    echo "wheel_bridge restarted"
REMOTE
fi

# --- HDMI monitor on fastgpu ---
echo "Launching monitor on fastgpu HDMI..."
ssh f "WAYLAND_DISPLAY=wayland-1 XDG_RUNTIME_DIR=/run/user/1000 \
  alacritty -e bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH; ~/src/experimental/polaris/target/release/tui_dashboard --monitor; echo EXITED; read'" &

# --- G923 wheel (takes over terminal) ---
echo ""
echo "Starting G923 wheel..."
echo "  Paddles = shift | Circle = handbrake | Options = e-stop | Q = quit"
echo ""
cd "$WORKSPACE"
exec cargo run --release -p polaris_wheel --bin polaris_wheel
