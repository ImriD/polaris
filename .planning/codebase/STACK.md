# Technology Stack

**Analysis Date:** 2026-02-23

## Languages

**Primary:**
- Rust 1.93.0 — all runtime binaries (5 crates: polaris_bridge, polaris_control, polaris_operator, polaris_sim, polaris_wheel)

**Secondary:**
- Python 3 — ROS2 launch files only (`launch/real.launch.py`, `launch/beamng.launch.py`, `launch/sitl.launch.py`)
- CMake — polaris_msgs build (`polaris_msgs/CMakeLists.txt`)

## Runtime

**Environment:**
- ROS2 Jazzy (Arch Linux on fastgpu, the deployment target)
- macOS (polaris_wheel only — Logitech G923 wheel client, runs locally on Mac)

**Package Manager:**
- Cargo 1.93.0
- Lockfile: `Cargo.lock` present and committed

## Frameworks

**Core:**
- `r2r` 0.9.5 — pure-Rust ROS2 client bindings; used in polaris_bridge, polaris_control, polaris_operator, polaris_sim. polaris_wheel intentionally has NO r2r dependency.
- `tokio` 1.49.0 (features = ["full"]) — async runtime; all nodes use `#[tokio::main]` or `tokio::runtime::Runtime::new()`

**TUI:**
- `ratatui` 0.29 — terminal UI framework; used in polaris_operator (tui_dashboard) and polaris_wheel
- `crossterm` 0.28 — terminal backend for ratatui; keyboard/raw mode handling

**Build/Dev:**
- `colcon` — builds polaris_msgs ROS2 package on fastgpu (separate from Cargo workspace)
- `cargo build --release` — builds all 5 Rust crates

## Key Dependencies

**Critical:**
- `mavlink` 0.14.1 (features = ["ardupilotmega", "direct-serial"]) — MAVLink protocol for Pixhawk communication; used exclusively in `polaris_bridge/src/pixhawk.rs`
- `quinn` 0.11.9 — QUIC transport for wheel→server link; used in polaris_operator (`wheel_bridge_node`) and polaris_wheel. Self-signed TLS certificates via `rcgen`
- `rcgen` 0.14.7 — generates self-signed TLS certs persisted at `~/.config/polaris/cert.der` and `~/.config/polaris/key.der`
- `sdl2` 0.38.0 (features = ["use-pkgconfig"]) — Logitech G923 joystick input via SDL2; macOS only (polaris_wheel). Requires SDL2 system library via pkg-config

**Middleware:**
- `rmp-serde` 1.x + `rmpv` 1.x — MessagePack serialization for BeamNG bridge protocol; used in `polaris_sim/src/bridge.rs`
- `serde` 1.x (features = ["derive"]) — serialization derive macros; used in polaris_sim
- `gilrs` 0.11 — cross-platform gamepad input; used in polaris_operator for local gamepad/G923 when directly connected to the operator machine (not macOS wheel path)
- `futures` 0.3 — StreamExt for ROS2 topic subscription patterns across all ROS2 crates

**Logging:**
- `log` 0.4 + `env_logger` 0.11 — all crates use `log::info!`, `log::warn!`, `log::error!`. Activated via `RUST_LOG=info` environment variable

## Configuration

**Environment Variables:**
- `PIXHAWK_ADDR` — MAVLink connection string for pixhawk_bridge_node. Default: `"serial:/dev/ttyUSB0:921600"`. SITL uses `tcpout:127.0.0.1:5762`, UDP SITL uses `udp:127.0.0.1:5762`
- `RUST_LOG` — log level filter (e.g. `info`)
- `LD_LIBRARY_PATH` — must include ROS2 shared libraries when running on fastgpu; set by sourcing `/opt/ros/jazzy/setup.bash`
- `SPEED_LIMIT` — accepted by vehicle_control_node (set in launch scripts, e.g. `SPEED_LIMIT=8`)

**ROS2 IDL Filter:**
- `.cargo/config.toml` sets `IDL_PACKAGE_FILTER = "polaris_msgs;std_msgs;std_srvs;sensor_msgs;geometry_msgs;builtin_interfaces"` — limits which ROS2 message packages r2r generates bindings for

**Build:**
- `Cargo.toml` (workspace root) — workspace resolver = "2", members: polaris_control, polaris_sim, polaris_bridge, polaris_operator, polaris_wheel
- `polaris_msgs/CMakeLists.txt` — ROS2 ament_cmake build for VehicleState.msg; built separately with `colcon build` on fastgpu in `~/ros2_ws`

## Platform Requirements

**Development (Mac):**
- Rust 1.93.0
- SDL2 system library (for polaris_wheel, via pkg-config)
- polaris_wheel runs locally: `cargo run --release -p polaris_wheel`

**Production (fastgpu — Arch Linux):**
- ROS2 Jazzy installed at `/opt/ros/jazzy/`
- polaris_msgs colcon workspace at `~/ros2_ws/`
- ArduPilot SITL at `~/ardupilot/` (for sim mode)
- mavproxy Python 3.12 venv at `~/mavproxy-venv/` (Python 3.14 default breaks mavproxy)
- Code deployed via `sync.sh` (rsync/scp to `f` SSH alias → `fastgpu.local`)

---

*Stack analysis: 2026-02-23*
