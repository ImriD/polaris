# External Integrations

**Analysis Date:** 2026-02-23

## Hardware & Protocol Integrations

**Pixhawk Autopilot (MAVLink):**
- What: Flight controller for real vehicle and ArduPilot SITL; used as vehicle actuator interface
- Protocol: MAVLink (ardupilotmega dialect)
- SDK: `mavlink` crate 0.14.1 with `ardupilotmega` + `direct-serial` features
- Connection: string-addressed via `PIXHAWK_ADDR` env var
  - Serial (real): `serial:/dev/ttyUSB0:921600`
  - SITL TCP: `tcpout:127.0.0.1:5762`
  - SITL UDP: `udp:127.0.0.1:5762`
- Implementation: `polaris_bridge/src/pixhawk.rs`
- Messages used:
  - Sends: `RC_CHANNELS_OVERRIDE` (5 channels: steering/brake/throttle/gear/handbrake), `COMMAND_LONG` (arm, set mode)
  - Sends: `REQUEST_DATA_STREAM` (10Hz telemetry stream)
  - Receives: `HEARTBEAT`, `GLOBAL_POSITION_INT`, `RAW_IMU`
- Mode: Forces MANUAL mode (`MAV_CMD_DO_SET_MODE`) and force-arms on connect

**Logitech G923 Racing Wheel (SDL2):**
- What: Physical steering wheel input device; macOS-only client
- SDK: `sdl2` 0.38.0 (via pkg-config)
- Implementation: `polaris_wheel/src/main.rs`
- Axes: Axis 0=steering, Axis 1=brake, Axis 2=throttle
- Buttons: R1(5)=gear up, L1(4)=gear down, Circle(1)=handbrake, Options(9)=e-stop
- Note: SDL2 must run on macOS main thread; `block_in_place` used for TUI loop

**Logitech G923 / Generic Gamepad (gilrs):**
- What: Alternative gamepad input path for operator machine (Linux/Windows); separate from polaris_wheel SDL2 path
- SDK: `gilrs` 0.11
- Implementation: `polaris_operator/src/gamepad.rs`, `polaris_operator/src/main.rs`, `polaris_operator/src/bin/gamepad_node.rs`
- Axes: LeftStickX=steering, LeftStickY=throttle, LeftZ=brake
- Buttons: RightTrigger/RightTrigger2=gear up, LeftTrigger/LeftTrigger2=gear down, East(Circle)=handbrake, Start=e-stop

**BeamNG Drive Simulator (MessagePack/TCP):**
- What: Vehicle simulator used as alternative to real Pixhawk; runs on fastgpu (Windows BeamNG)
- Protocol: Custom length-prefixed MessagePack over TCP
- SDK: `rmpv` + `rmp-serde`
- Connection: TCP to `fastgpu:64256` (hardcoded in `polaris_sim/src/main.rs`)
- Protocol version: `v1.26` (handshake verified on connect)
- Implementation: `polaris_sim/src/bridge.rs`
- Message flow:
  1. Connect to GE (game engine) port 64256
  2. `Hello` handshake with protocol version check
  3. `GetCurrentVehicles` → get vehicle ID
  4. `StartVehicleConnection` → get vehicle-specific port
  5. Connect second TCP stream to vehicle port
  6. `QueueLuaCommandVE` — disables player input
  7. Loop: send `Control` message (steering/throttle/brake/parkingbrake/gear), receive `SensorRequest` response (pos, vel, rotation)
- Wire format: `[u32 big-endian length][msgpack payload]` with auto-incrementing `_id` field

## Transport Layer

**QUIC (wheel → server link):**
- What: Bidirectional reliable transport from Mac (polaris_wheel) to fastgpu (wheel_bridge_node); replaces ROS2 network for the remote operator path
- SDK: `quinn` 0.11.9 (QUIC over UDP, TLS via rustls)
- Port: 9876 (server-side, `wheel_bridge_node`)
- TLS: Self-signed certificates, persisted at `~/.config/polaris/cert.der` and `~/.config/polaris/key.der`. Client (`polaris_wheel`) uses `SkipServerVerification` — no cert validation
- Packet format (client→server, 14 bytes): `[f32 steering][f32 throttle][f32 brake][u8 gear][u8 handbrake]`
- Packet format (server→client, 72 bytes): Full `VehicleDown` struct (speed, heading, lat, lon, safe actuators, mode, altitude, IMU, GPS)
- Frequency: 50Hz (20ms interval)
- Implementation: `polaris_operator/src/bin/wheel_bridge_node.rs` (server), `polaris_wheel/src/main.rs` (client)
- Reconnect: Client auto-reconnects with 1s backoff; server accepts multiple connections

## ROS2 Middleware

**ROS2 Jazzy:**
- What: Pub/sub middleware connecting all nodes on fastgpu
- SDK: `r2r` 0.9.5 (pure Rust, no rclcpp)
- Topics published/subscribed: see ARCHITECTURE.md
- Custom message: `polaris_msgs/msg/VehicleState.msg` — built via colcon from `polaris_msgs/CMakeLists.txt`, installed to `~/ros2_ws/install/`
- Standard messages used: `std_msgs/Float32`, `std_msgs/UInt8`, `std_msgs/Bool`, `std_msgs/String`, `sensor_msgs/Imu`, `sensor_msgs/NavSatFix`

## Remote Deployment

**SSH to fastgpu:**
- SSH alias: `f` → `fastgpu.local` (user: `mates` per project memory)
- Sync: `sync.sh` uses `scp -r` to copy source tree; does NOT delete removed files
- Build on remote: `cargo build --release -p polaris_bridge -p polaris_operator -p polaris_sim`
- Binaries deployed to: `~/src/experimental/polaris/target/release/`

## ArduPilot SITL

**sim_vehicle.py:**
- What: ArduPilot software-in-the-loop Rover simulation
- Location: `~/ardupilot/Tools/autotest/sim_vehicle.py` on fastgpu
- Requires: mavproxy Python 3.12 venv (`~/mavproxy-venv/`) — Python 3.14 breaks SITL clock
- Launch wrapper: `/tmp/run_pty.py` PTY script (mavproxy requires real TTY)
- Ports: SERIAL0 at TCP 5760 (mavproxy), SERIAL1 at TCP 5762 (pixhawk_bridge_node)
- Startup delay: 20 seconds for SITL to initialize before bridge connects

## Monitoring / Display

**alacritty (HDMI monitor):**
- What: Terminal emulator launched on fastgpu HDMI display for monitor mode
- Launch: `WAYLAND_DISPLAY=wayland-1 XDG_RUNTIME_DIR=/run/user/1000 alacritty -e bash -c '... tui_dashboard --monitor'`
- Not a code integration — external system dependency on fastgpu

## Data Storage

**Databases:** None

**File Storage:**
- TLS certificates: `~/.config/polaris/cert.der`, `~/.config/polaris/key.der` — generated on first run by `wheel_bridge_node`, loaded on subsequent runs
- SITL logs: `/tmp/sim_vehicle.log`, `/tmp/pixhawk_bridge.log`, `/tmp/vehicle_control.log`, `/tmp/wheel_bridge.log` — written with `nohup` on fastgpu

**Caching:** None

## Authentication & Identity

**Auth Provider:** None (no web auth)
- QUIC link uses self-signed TLS cert for transport encryption only; no authentication — any client connecting to port 9876 is accepted

## Monitoring & Observability

**Error Tracking:** None

**Logs:**
- `env_logger` + `log` crate across all Rust binaries; level controlled by `RUST_LOG` env var
- nohup log files on fastgpu for each node (see File Storage above)
- `tui_dashboard` provides live 1Hz Hz-rate display for all topics

## CI/CD & Deployment

**Hosting:** Arch Linux server (`fastgpu.local`) — local network only

**CI Pipeline:** None

**Deploy process:** `./launch/sim.sh` — syncs code, builds on remote, launches SITL + all nodes

---

*Integration audit: 2026-02-23*
