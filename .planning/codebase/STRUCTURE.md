# Codebase Structure

**Analysis Date:** 2026-02-23

## Directory Layout

```
polaris/                        # Cargo workspace root
├── Cargo.toml                  # Workspace manifest (5 members)
├── Cargo.lock                  # Lockfile (committed)
├── SPEC.md                     # Project specification
├── README.md                   # Setup and architecture overview
├── sync.sh                     # rsync workspace to fastgpu + build
├── launch/
│   ├── sim.sh                  # Full launch: sync + SITL + nodes + wheel
│   └── hdmi.sh                 # Launch TUI on fastgpu HDMI display
├── polaris_msgs/               # ROS2 message package (colcon, not in Cargo workspace)
│   ├── msg/
│   │   └── VehicleState.msg    # Custom telemetry message definition
│   ├── CMakeLists.txt          # colcon build config (deleted on this branch)
│   └── package.xml             # ROS2 package manifest (deleted on this branch)
├── polaris_wheel/              # Mac-only: G923 wheel app (SDL2 + QUIC, no ROS2)
│   ├── Cargo.toml
│   └── src/
│       ├── main.rs             # Binary: polaris_wheel
│       └── test_detect.rs      # SDL2 device detection helper
├── polaris_operator/           # fastgpu: TUI dashboard + input bridge nodes
│   ├── Cargo.toml
│   └── src/
│       ├── main.rs             # Binary: tui_dashboard (ratatui + ROS2)
│       ├── gamepad.rs          # Shared gamepad normalization module
│       └── bin/
│           ├── wheel_bridge_node.rs  # Binary: wheel_bridge_node (QUIC → /input/*)
│           └── gamepad_node.rs       # Binary: gamepad_node (gilrs → /input/*)
├── polaris_control/            # fastgpu: /input/* → /cmd/* passthrough at 50Hz
│   ├── Cargo.toml
│   └── src/
│       └── main.rs             # Binary: vehicle_control_node
├── polaris_bridge/             # fastgpu: Pixhawk MAVLink bridge
│   ├── Cargo.toml
│   └── src/
│       ├── main.rs             # Binary: pixhawk_bridge_node
│       └── pixhawk.rs          # PixhawkInterface — MAVLink session wrapper
├── polaris_sim/                # fastgpu: BeamNG MessagePack bridge
│   ├── Cargo.toml
│   └── src/
│       ├── main.rs             # Binary: beamng_bridge_node
│       └── bridge.rs           # BeamNGBridge — TCP/MessagePack client
├── .cargo/
│   └── config.toml             # Cargo workspace config
└── .planning/
    └── codebase/               # Architecture docs (this directory)
```

## Directory Purposes

**`polaris_msgs/`:**
- Purpose: ROS2 message definitions; built separately with colcon, not part of Cargo workspace
- Contains: `VehicleState.msg` — the single shared message type
- Key files: `polaris_msgs/msg/VehicleState.msg`
- Note: Must be symlinked into `~/ros2_ws/src/` on fastgpu and built with `colcon build --packages-select polaris_msgs`

**`polaris_wheel/`:**
- Purpose: Standalone Mac binary; the only crate that runs on the operator's machine
- Contains: SDL2 G923 driver, QUIC client, ratatui TUI
- Key files: `polaris_wheel/src/main.rs`
- No ROS2 dependency — communicates exclusively via QUIC

**`polaris_operator/`:**
- Purpose: All operator-facing ROS2 nodes; three binaries from one crate
- Contains: TUI dashboard, QUIC→ROS2 wheel bridge, standalone gamepad node
- Key files: `polaris_operator/src/main.rs` (tui_dashboard), `polaris_operator/src/bin/wheel_bridge_node.rs`, `polaris_operator/src/bin/gamepad_node.rs`, `polaris_operator/src/gamepad.rs`

**`polaris_control/`:**
- Purpose: The rate-normalization node; keeps `/cmd/*` publishing at exactly 50Hz regardless of input rate
- Contains: Single binary with 5 subscriptions and 5 publications
- Key files: `polaris_control/src/main.rs`

**`polaris_bridge/`:**
- Purpose: Pixhawk/ArduPilot interface — works with both real hardware and SITL
- Contains: MAVLink RC override sender, telemetry receiver, connection retry
- Key files: `polaris_bridge/src/main.rs`, `polaris_bridge/src/pixhawk.rs`

**`polaris_sim/`:**
- Purpose: BeamNG Drive simulator interface via proprietary MessagePack TCP API
- Contains: BeamNG RPC client, vehicle attachment, sensor polling
- Key files: `polaris_sim/src/main.rs`, `polaris_sim/src/bridge.rs`

**`launch/`:**
- Purpose: Shell scripts to orchestrate the full system
- Key files: `launch/sim.sh` (primary launch), `launch/hdmi.sh` (HDMI monitor)

## Key File Locations

**Entry Points:**
- `polaris_wheel/src/main.rs`: Mac wheel binary (SDL2 + QUIC client)
- `polaris_operator/src/main.rs`: Operator TUI dashboard
- `polaris_operator/src/bin/wheel_bridge_node.rs`: QUIC server + ROS2 bridge (primary input source)
- `polaris_operator/src/bin/gamepad_node.rs`: Standalone gamepad node (alternative input)
- `polaris_control/src/main.rs`: Command passthrough node
- `polaris_bridge/src/main.rs`: Pixhawk MAVLink node
- `polaris_sim/src/main.rs`: BeamNG bridge node

**Configuration:**
- `Cargo.toml`: Workspace members list
- `sync.sh`: Remote build configuration (target machine, paths)
- `launch/sim.sh`: Full system launch (SITL settings, binary paths, env vars)

**Core Logic:**
- `polaris_bridge/src/pixhawk.rs`: MAVLink protocol, RC channel PWM mapping
- `polaris_sim/src/bridge.rs`: BeamNG MessagePack protocol, connection state machine
- `polaris_operator/src/gamepad.rs`: Gamepad axis normalization, deadzone, gear logic
- `polaris_msgs/msg/VehicleState.msg`: Shared data contract between all nodes

**ROS2 Message Package:**
- `polaris_msgs/msg/VehicleState.msg`: The only custom message type in the system

## Naming Conventions

**Files:**
- Rust source: `snake_case.rs`
- Binary entry points: `main.rs` for single-binary crates; named files in `src/bin/` for multi-binary crates
- Protocol modules: named after what they talk to (`pixhawk.rs`, `bridge.rs`, `gamepad.rs`)

**Binaries (final names):**
- ROS2 nodes: `*_node` suffix — `pixhawk_bridge_node`, `vehicle_control_node`, `wheel_bridge_node`, `beamng_bridge_node`, `gamepad_node`
- User-facing apps: descriptive — `tui_dashboard`, `polaris_wheel`

**Crates:**
- `polaris_<role>` pattern — `polaris_bridge`, `polaris_control`, `polaris_operator`, `polaris_sim`, `polaris_wheel`

**ROS2 Topics:**
- `/input/<actuator>` — raw operator commands (from wheel_bridge_node or tui_dashboard)
- `/cmd/<actuator>` — rate-normalized commands (from vehicle_control_node)
- `/vehicle/<sensor>` — vehicle feedback (from bridge nodes)
- Actuator names: `steering`, `throttle`, `brake`, `gear`, `handbrake`
- Sensor names: `state`, `imu`, `gps`, `mode`

**Internal State Structs:**
- Each node defines its own local state struct: `DashboardState`, `WheelState`, `CmdState`, `Actuators`, `VehicleDown`
- No shared library types — each crate redefines its own version of common shapes

## Where to Add New Code

**New ROS2 node (on fastgpu):**
- Create new crate: `polaris_<name>/Cargo.toml` + `polaris_<name>/src/main.rs`
- Add to `Cargo.toml` workspace `members`
- Add binary name `*_node` in `[[bin]]` section of crate's `Cargo.toml`
- Launch: add `nohup $BIN/<node_name> > /tmp/<node_name>.log 2>&1 &` to `launch/sim.sh`

**New binary in existing crate:**
- Add `[[bin]]` entry in the crate's `Cargo.toml`
- Add `src/bin/<binary_name>.rs`
- Example pattern: see `polaris_operator/src/bin/wheel_bridge_node.rs`

**New ROS2 message field:**
- Edit `polaris_msgs/msg/VehicleState.msg`
- Rebuild on fastgpu: `cd ~/ros2_ws && colcon build --packages-select polaris_msgs`
- Update all nodes that use the field (all bridge nodes publish it, dashboard consumes it)

**New hardware bridge (alternative to Pixhawk/BeamNG):**
- Create `polaris_<hardware>/` crate following `polaris_bridge` pattern
- Subscribe `/cmd/*`, publish `/vehicle/state`
- Add `LD_LIBRARY_PATH` sourcing in launch script (required for ROS2 `.so` resolution)

**Shared utility logic:**
- If used by one crate only: add as a module `src/<name>.rs` in that crate
- If used by multiple crates on fastgpu: `polaris_operator/src/gamepad.rs` is the current pattern for shared modules (included via `#[path]` in sibling binaries within the same crate)
- There is no shared library crate currently; avoid creating one unless truly needed by 3+ crates

## Special Directories

**`polaris_msgs/`:**
- Purpose: ROS2 colcon package, not a Cargo crate
- Generated: No (hand-written .msg files)
- Committed: Yes

**`target/`:**
- Purpose: Cargo build output
- Generated: Yes
- Committed: No

**`.planning/`:**
- Purpose: Architecture documentation and planning files
- Generated: No
- Committed: Yes

**`~/.config/polaris/` (runtime, on fastgpu/Mac):**
- Purpose: TLS certificates for QUIC (`cert.der`, `key.der`) — auto-generated on first run
- Generated: Yes (by `wheel_bridge_node` using `rcgen`)
- Committed: No
