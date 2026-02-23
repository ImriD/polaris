# Architecture

**Analysis Date:** 2026-02-23

## Pattern Overview

**Overall:** Pipeline Node Architecture over ROS2 pub/sub

**Key Characteristics:**
- Each crate is a standalone binary (ROS2 node or standalone app) with no shared library code
- Commands flow unidirectionally through a fixed pipeline: input → control → bridge
- Telemetry flows in reverse on `/vehicle/*` topics, consumed by all display nodes
- Two transport layers coexist: ROS2 DDS (on fastgpu) and QUIC (cross-machine, Mac → fastgpu)
- All ROS2 nodes use `r2r` (Rust bindings) with `tokio` single-thread executor pattern

## Layers

**Input Layer:**
- Purpose: Capture operator intent from physical devices and publish as `/input/*` topics
- Location: `polaris_operator/src/main.rs` (tui_dashboard), `polaris_operator/src/bin/wheel_bridge_node.rs`, `polaris_operator/src/bin/gamepad_node.rs`, `polaris_wheel/src/main.rs`
- Contains: SDL2 axis reading, gilrs gamepad polling, QUIC client/server, TUI rendering
- Depends on: SDL2, gilrs, quinn (QUIC), ratatui
- Used by: vehicle_control_node (consumes `/input/*`)

**Control Layer:**
- Purpose: Passthrough and rate-normalization from `/input/*` to `/cmd/*` at 50Hz
- Location: `polaris_control/src/main.rs`
- Contains: 5 subscribers (steering, throttle, brake, gear, handbrake), 5 publishers, 50Hz wall timer
- Depends on: r2r, tokio
- Used by: bridge nodes (consume `/cmd/*`)

**Bridge Layer:**
- Purpose: Translate `/cmd/*` ROS2 topics into hardware/simulator protocols
- Location: `polaris_bridge/src/main.rs` + `polaris_bridge/src/pixhawk.rs` (real/SITL Pixhawk), `polaris_sim/src/main.rs` + `polaris_sim/src/bridge.rs` (BeamNG)
- Contains: MAVLink RC_CHANNELS_OVERRIDE, BeamNG MessagePack TCP protocol, retry loops
- Depends on: mavlink crate, rmpv/rmp-serde, tokio threads
- Used by: publishes `/vehicle/state`, `/vehicle/imu`, `/vehicle/gps`

**Telemetry Layer:**
- Purpose: Propagate vehicle state back to operator displays
- Topics: `/vehicle/state` (VehicleState custom msg), `/vehicle/imu` (sensor_msgs/Imu), `/vehicle/gps` (sensor_msgs/NavSatFix)
- Consumed by: tui_dashboard, wheel_bridge_node (re-serializes over QUIC back to polaris_wheel)

## Data Flow

**Command Flow (operator → vehicle):**

1. Physical input: G923 wheel (SDL2 on Mac in `polaris_wheel/src/main.rs`) or gamepad/keyboard (gilrs in `polaris_operator/src/main.rs`)
2. QUIC transport: `polaris_wheel` sends 14-byte binary packet at 50Hz to `wheel_bridge_node` on fastgpu port 9876
3. `/input/*` topics: `wheel_bridge_node` or `tui_dashboard` publishes 5 std_msgs topics (Float32 steering/throttle/brake, UInt8 gear, Bool handbrake)
4. `vehicle_control_node` subscribes to `/input/*`, stores latest values, re-publishes to `/cmd/*` at exactly 50Hz via wall timer
5. `pixhawk_bridge_node` or `beamng_bridge_node` subscribes to `/cmd/*`, sends RC_CHANNELS_OVERRIDE (MAVLink) or Control (BeamNG MessagePack) to hardware

**Telemetry Flow (vehicle → operator):**

1. Bridge nodes receive GLOBAL_POSITION_INT + RAW_IMU from Pixhawk, or SensorRequest response from BeamNG
2. Publish `/vehicle/state` (VehicleState), `/vehicle/imu`, `/vehicle/gps`
3. `wheel_bridge_node` subscribes, packs into 72-byte struct, sends back over same QUIC bidirectional stream
4. `polaris_wheel` receives 72-byte packet, updates TUI display
5. `tui_dashboard` subscribes to all `/vehicle/*` topics directly via ROS2

**QUIC Wire Protocol:**
- Uplink: 14 bytes little-endian — `[f32 steering][f32 throttle][f32 brake][u8 gear][u8 handbrake]`
- Downlink: 72 bytes little-endian — speed, heading, lat, lon, safe actuators, gear, mode, altitude, IMU (acc+gyro)
- Bidirectional stream: client writes 14 bytes, server writes 72 bytes, repeat at 50Hz

**State Management:**
- Each node holds its own `Arc<Mutex<State>>` struct (no shared memory between processes)
- Bridge nodes accumulate latest `/cmd/*` values into a local struct, apply on each telemetry tick
- `tui_dashboard` tracks `TopicRates` via `AtomicU64` counters, sampled every 1s to compute Hz

## Key Abstractions

**VehicleState message:**
- Purpose: Unified telemetry snapshot from any bridge (real or simulated)
- Definition: `polaris_msgs/msg/VehicleState.msg`
- Fields: steering_angle, throttle_position, brake_pressure, gear_current, handbrake_engaged, speed_mps, heading_deg, lat, lon, mode (0=MANUAL/1=REMOTE/2=AUTO), stamp
- Built with colcon (ROS2 build system); consumed by r2r as `r2r::polaris_msgs::msg::VehicleState`

**PixhawkInterface:**
- Purpose: MAVLink session wrapper — connect, arm, set mode, send RC override, recv telemetry
- Location: `polaris_bridge/src/pixhawk.rs`
- Pattern: Blocking `MavConnection` used in a dedicated `std::thread::spawn` (MAVLink recv is blocking)
- Returns `TelemetryUpdate` enum: `Position{lat,lon,alt,heading_deg,speed_mps}` or `Imu{acc,gyro}`

**BeamNGBridge:**
- Purpose: Stateful TCP client for BeamNG's MessagePack RPC protocol
- Location: `polaris_sim/src/bridge.rs`
- Pattern: Async; two TCP streams (GE socket + vehicle socket), length-prefixed MessagePack frames
- State machine: `Disconnected → Connected → Ready → Error`

**GamepadState:**
- Purpose: Shared gamepad normalization logic (deadzone, gear shifting, pedal mapping)
- Location: `polaris_operator/src/gamepad.rs`
- Used by: `tui_dashboard` (via `mod gamepad`) and `gamepad_node` (via `#[path]` include)

## Entry Points

**polaris_wheel (Mac binary):**
- Location: `polaris_wheel/src/main.rs`, binary `polaris_wheel`
- Triggers: `cargo run --release -p polaris_wheel` or `./target/release/polaris_wheel <host:port>`
- Responsibilities: SDL2 G923 input → QUIC client → TUI display; runs entirely on Mac, no ROS2

**wheel_bridge_node (fastgpu):**
- Location: `polaris_operator/src/bin/wheel_bridge_node.rs`
- Triggers: launched by `launch/sim.sh` via nohup on fastgpu
- Responsibilities: QUIC server port 9876 → `/input/*` ROS2 topics; also subscribes to `/vehicle/*` and `/cmd/*` to send telemetry back over QUIC

**vehicle_control_node (fastgpu):**
- Location: `polaris_control/src/main.rs`
- Triggers: launched by `launch/sim.sh`
- Responsibilities: Pure passthrough `/input/*` → `/cmd/*` at 50Hz; the rate-normalization point

**pixhawk_bridge_node (fastgpu):**
- Location: `polaris_bridge/src/main.rs`
- Triggers: launched with `PIXHAWK_ADDR=tcpout:127.0.0.1:5762` env var for SITL
- Responsibilities: MAVLink connection with retry, RC override sending, telemetry publishing

**beamng_bridge_node (fastgpu):**
- Location: `polaris_sim/src/main.rs`
- Triggers: manual launch when using BeamNG sim
- Responsibilities: BeamNG MessagePack protocol, vehicle attachment, sensor polling

**tui_dashboard (fastgpu, SSH or HDMI):**
- Location: `polaris_operator/src/main.rs`
- Modes: normal (publishes `/input/*` when local gamepad present), `--monitor` (subscribe-only)
- Responsibilities: ratatui TUI showing all topic Hz, actuator bars, telemetry; keyboard/gamepad input fallback

## Error Handling

**Strategy:** Let it crash and retry. All bridge nodes have auto-reconnect loops.

**Patterns:**
- Pixhawk: `connect_timer` fires every 2s; if `pixhawk: Arc<Mutex<Option<PixhawkInterface>>>` is `None`, attempt reconnect
- BeamNG: `if bridge.state() != Ready { bridge.disconnect(); bridge.connect().await }` loop with 2s sleep on error
- QUIC server: `while let Some(incoming) = endpoint.accept().await` — each connection gets its own `tokio::spawn`
- QUIC client: `loop { connect → session → on error: sleep(1s) → retry }`
- `Result` errors are logged via `log::warn!` and discarded with `let _ = ...` for non-critical publish calls

## Cross-Cutting Concerns

**Logging:** `env_logger` in all nodes; level controlled by `RUST_LOG=info` env var
**Validation:** No explicit input validation; values are clamped at protocol boundaries (e.g. `clamp(-1.0, 1.0)` before MAVLink)
**Authentication:** QUIC uses self-signed TLS cert stored at `~/.config/polaris/{cert,key}.der`; client skips server cert verification (`SkipServerVerification`)
**Threading:** All ROS2 nodes use `tokio::task::LocalSet` with `spawn_local` (single-thread executor); bridge telemetry threads use `std::thread::spawn` for blocking MAVLink recv
