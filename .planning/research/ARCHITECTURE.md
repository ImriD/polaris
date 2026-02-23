# Architecture — Jetson ROS2 Deployment

**Research Date:** 2026-02-23
**Scope:** How to deploy existing ROS2 nodes onto a Jetson while keeping the fastgpu sim setup working.

---

## Summary

The existing codebase already has the right shape for dual-target deployment. The command pipeline is environment-agnostic — same Rust binaries, same topic names, same wire protocol. The only things that differ between sim (fastgpu) and hardware (Jetson) are: which binary connects to the Pixhawk, how that connection is addressed, and what launch script brings it up. No code changes required for the core pipeline; the adaptation surface is a single env var and a new launch script.

---

## Component Boundaries

### Mac (192.168.88.108) — no ROS2

```
polaris_wheel
  - SDL2 reads G923 wheel axes and buttons
  - QUIC client connects to port 9876 on the compute machine (fastgpu or Jetson)
  - Sends 14-byte command packets at 50Hz
  - Receives 72-byte telemetry packets
  - Renders TUI display locally
```

This component is identical for both targets. The only configuration is the server address (currently hardcoded to fastgpu — needs to be an arg or env var for Jetson).

### Jetson (192.168.88.175) — ROS2 Humble, aarch64, Ubuntu 22.04

```
wheel_bridge_node  (polaris_operator)
  - QUIC server port 9876
  - Ingests 14-byte command packets → publishes /input/* (5 topics)
  - Subscribes to /vehicle/* → packs 72-byte telemetry → sends back over QUIC

vehicle_control_node  (polaris_control)
  - Subscribes /input/* (5 topics)
  - Re-publishes /cmd/* at exactly 50Hz via wall timer
  - Rate normalization point

pixhawk_bridge_node  (polaris_bridge)
  - Subscribes /cmd/* (5 topics)
  - PIXHAWK_ADDR=udpin:0.0.0.0:14550 (or tcpout:192.168.88.242:<port>)
  - Sends RC_CHANNELS_OVERRIDE to real Pixhawk over Ethernet MAVLink
  - Publishes /vehicle/state, /vehicle/imu, /vehicle/gps

tui_dashboard  (polaris_operator, --monitor flag)
  - Subscribes all /vehicle/* and /input/* and /cmd/* topics
  - Renders live Hz rates and actuator bars
  - No HDMI on Jetson — access over SSH with TERM set
```

Not on Jetson:
- `polaris_sim` — BeamNG bridge stays on fastgpu only
- SITL — no ArduPilot sim, no mavproxy, no PTY wrapper

### Pixhawk (192.168.88.242) — Ethernet-native MAVLink

```
  - Receives RC_CHANNELS_OVERRIDE from pixhawk_bridge_node
  - Sends HEARTBEAT, GLOBAL_POSITION_INT, RAW_IMU at 10Hz
  - Connection via UDP or TCP over the 192.168.88.x LAN
```

### fastgpu — unchanged sim path

```
  - Runs its own copy of wheel_bridge_node, vehicle_control_node, pixhawk_bridge_node (SITL)
  - SITL: sim_vehicle.py + mavproxy at TCP 5762 (loopback)
  - beamng_bridge_node available for BeamNG mode
  - launch/sim.sh remains the entry point
```

---

## Data Flow

### Command path (operator → Pixhawk)

```
G923 wheel (Mac)
  [SDL2 axis/button read]
  → polaris_wheel encodes 14-byte packet
  → QUIC UDP to 192.168.88.175:9876

wheel_bridge_node (Jetson)
  → decodes packet
  → publishes /input/steering (Float32)
  → publishes /input/throttle (Float32)
  → publishes /input/brake    (Float32)
  → publishes /input/gear     (UInt8)
  → publishes /input/handbrake (Bool)

vehicle_control_node (Jetson)
  → subscribes /input/*
  → publishes /cmd/* at 50Hz wall-timer tick

pixhawk_bridge_node (Jetson)
  → subscribes /cmd/*
  → accumulates latest values in CmdState
  → on each MAVLink telemetry tick: sends RC_CHANNELS_OVERRIDE
  → MAVLink UDP/TCP → Pixhawk at 192.168.88.242
```

### Telemetry path (Pixhawk → operator)

```
Pixhawk
  → MAVLink GLOBAL_POSITION_INT, RAW_IMU at 10Hz

pixhawk_bridge_node (Jetson)
  → parses MAVLink frames (blocking thread)
  → publishes /vehicle/state  (VehicleState)
  → publishes /vehicle/gps    (NavSatFix)
  → publishes /vehicle/imu    (Imu)

wheel_bridge_node (Jetson)
  → subscribes /vehicle/*
  → encodes 72-byte VehicleDown struct
  → sends back over same QUIC bidirectional stream

polaris_wheel (Mac)
  → receives 72-byte packet
  → updates TUI display (speed, heading, lat/lon, actuator echo)

tui_dashboard (Jetson, SSH session)
  → subscribes /vehicle/* + /input/* + /cmd/*
  → renders live Hz and actuator bars
```

### ROS2 DDS scope

All ROS2 pub/sub stays local to the Jetson. DDS does not span the Teltonika network to the Mac — that boundary is the QUIC link on port 9876. fastgpu has its own isolated ROS2 domain; no cross-machine ROS2 needed.

---

## Key Differences: Jetson vs fastgpu

| Aspect | fastgpu (sim) | Jetson (hardware) |
|--------|---------------|-------------------|
| OS | Arch Linux | Ubuntu 22.04 |
| ROS2 | Jazzy | Humble |
| Architecture | x86_64 | aarch64 |
| Build | Remote compile via sync.sh | Native compile on-device |
| PIXHAWK_ADDR | tcpout:127.0.0.1:5762 | udpin:0.0.0.0:14550 or tcpout:192.168.88.242:PORT |
| Bridge binary | pixhawk_bridge_node (SITL TCP) | pixhawk_bridge_node (Ethernet MAVLink) |
| Sim binary | beamng_bridge_node / pixhawk (SITL) | not deployed |
| Launch script | launch/sim.sh | launch/rig.sh (new) |
| SITL | sim_vehicle.py + mavproxy | none |
| Monitor display | alacritty on HDMI | tui_dashboard via SSH |
| polaris_wheel target | fastgpu IP / hostname | 192.168.88.175 or env var |

The r2r crate works with both Humble and Jazzy — no source changes needed for the ROS2 version difference. r2r links against the system ROS2 at build time via pkg-config.

---

## Sync and Build Approach

### Current (fastgpu)

`sync.sh` uses `scp -r` to push source from Mac → fastgpu, then `ssh f "cargo build --release ..."`. This works because fastgpu is x86_64 and has a persistent workspace.

### For Jetson

Same pattern applies. A new sync target in `sync.sh` (or a parallel `sync-rig.sh`) uses `scp -r` to push source to `nvidia@192.168.88.175:~/polaris/`. Build runs natively on the Jetson — no cross-compilation. The Jetson has 16GB RAM and a fast NVMe, so native compilation is practical.

Key build prerequisite: the Jetson must have:
1. ROS2 Humble installed (`/opt/ros/humble/`)
2. `polaris_msgs` colcon workspace built in `~/ros2_ws/`
3. Rust toolchain installed (rustup, stable)
4. r2r build dependencies: `libclang-dev`, `cmake`, `python3-colcon-common-extensions`

The colcon step (`colcon build --packages-select polaris_msgs`) must run once on the Jetson to generate the VehicleState message headers and the install tree that r2r links against. After that, `cargo build --release` handles everything.

---

## Launch Architecture (Table Rig)

A new `launch/rig.sh` script is the table-rig equivalent of `launch/sim.sh`. Structure mirrors the existing script but removes all SITL machinery:

```bash
# launch/rig.sh sketch

# 1. sync source to Jetson
scp -r ... nvidia@192.168.88.175:~/polaris/

# 2. build on Jetson
ssh jetson "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cargo build --release -p polaris_bridge -p polaris_operator -p polaris_control"

# 3. kill stale processes
ssh jetson "pkill -f 'pixhawk_bridge|vehicle_control|wheel_bridge|tui_dashboard' || true"

# 4. start nodes (no SITL, real Pixhawk address)
ssh jetson "
  PIXHAWK_ADDR=udpin:0.0.0.0:14550 nohup .../pixhawk_bridge_node > /tmp/pixhawk_bridge.log 2>&1 &
  SPEED_LIMIT=3 nohup .../vehicle_control_node > /tmp/vehicle_control.log 2>&1 &
  nohup .../wheel_bridge_node > /tmp/wheel_bridge.log 2>&1 &
"

# 5. start G923 wheel on Mac, pointing at Jetson
POLARIS_SERVER=192.168.88.175:9876 cargo run --release -p polaris_wheel
```

The Pixhawk MAVLink address format depends on how the Pixhawk is configured:
- If Pixhawk acts as UDP server: `udpin:0.0.0.0:14550` (Jetson listens, Pixhawk connects)
- If Pixhawk acts as UDP client: `udpout:192.168.88.242:14550` (Jetson sends to Pixhawk)
- TCP: `tcpout:192.168.88.242:5760`

The `mavlink` crate's connection string handles all three — only the `PIXHAWK_ADDR` env var changes.

---

## Build Order (Dependencies)

The dependency graph determines what must succeed before the next step:

```
1. polaris_msgs  (colcon build — generates VehicleState.msg headers)
        ↓
2. polaris_control   (only depends on r2r + std_msgs, no custom msgs produced)
   polaris_bridge    (depends on polaris_msgs via r2r bindings)
   polaris_operator  (depends on polaris_msgs via r2r bindings)
        ↓
3. launch/rig.sh can start nodes
        ↓
4. polaris_wheel (Mac, built separately — no ROS2 dep, just cargo)
```

Within step 2, the three crates can build in parallel (`cargo build --release` handles this automatically). polaris_sim is excluded from the Jetson build — it has no role there and its BeamNG TCP dependency doesn't apply.

Start order at runtime follows the pipeline:

```
1. pixhawk_bridge_node  — must be up before vehicle state can flow
2. vehicle_control_node — must be up before /cmd/* topics exist
3. wheel_bridge_node    — must be up before Mac wheel connects
4. tui_dashboard        — monitoring only, can start any time
5. polaris_wheel (Mac)  — operator endpoint, connects to wheel_bridge_node
```

Nodes auto-reconnect so strict ordering matters less than it appears — but this ordering avoids noisy startup warnings.

---

## Implications for Code

### polaris_wheel server address

Currently the QUIC server address in `polaris_wheel` is hardcoded or defaults to `fastgpu.local:9876`. For the table rig it needs to be `192.168.88.175:9876`. This should be a CLI arg or `POLARIS_SERVER` env var — one small change with no architectural impact.

### PIXHAWK_ADDR for Ethernet MAVLink

The `pixhawk_bridge_node` already reads `PIXHAWK_ADDR` from the environment with a sensible default. No code change needed — only the env var in the launch script changes.

### ROS2 Humble vs Jazzy

r2r 0.9.5 is compatible with both. The `.cargo/config.toml` IDL filter is version-agnostic. The only difference is the install path (`/opt/ros/humble/` vs `/opt/ros/jazzy/`) which is handled by sourcing the correct setup.bash in the launch script.

### polaris_msgs colcon workspace

The colcon workspace on the Jetson must be built once before `cargo build` can succeed (r2r generates Rust bindings from the installed message headers at build time). This is a one-time provisioning step, not an ongoing concern.

---

## What Does Not Change

- All ROS2 topic names and types — identical on both machines
- All Rust source code for the four deployed crates — no platform ifdefs needed
- The QUIC wire protocol — 14-byte uplink, 72-byte downlink, 50Hz
- The MAVLink command set — RC_CHANNELS_OVERRIDE with the same 5 channels
- The fastgpu sim workflow — `launch/sim.sh` and `sync.sh` untouched
