#!/bin/bash
set -euo pipefail
PI=polaris@10.42.0.2
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VERSION=$(git -C "$SCRIPT_DIR/.." rev-parse --short HEAD)

ssh "$PI" "mkdir -p ~/motor_sim ~/.config/systemd/user"
scp "$SCRIPT_DIR/motor_emulator.py" "$SCRIPT_DIR/motor-emulator.service" "$PI:~/motor_sim/"
ssh "$PI" "cp ~/motor_sim/motor-emulator.service ~/.config/systemd/user/ && loginctl enable-linger polaris && systemctl --user daemon-reload && systemctl --user enable --now motor-emulator"
echo "deployed $VERSION → $PI"
