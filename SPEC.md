# Polaris — ROS2 Vehicle Control

Remote-operated Polaris side-by-side over ROS2. Drive from a Mac with a G923 racing wheel, commands flow over QUIC to fastgpu which runs all ROS2 nodes and bridges to a Pixhawk autopilot (real or SITL).

## Architecture

6 Rust crates with clean topic boundaries:

- **polaris_msgs** — VehicleState message type (colcon)
- **polaris_wheel** — G923 wheel app on Mac (SDL2 + QUIC client, no ROS2)
- **polaris_operator** — Operator TUI, gamepad input, wheel QUIC↔ROS2 bridge
- **polaris_control** — Passthrough: /input/* → /cmd/* at 50Hz
- **polaris_bridge** — Pixhawk MAVLink interface (real + SITL)
- **polaris_sim** — BeamNG MessagePack bridge

## Data Flow

Separate topics per actuator (5 each):
- `/input/*` — operator commands (from wheel_bridge_node or tui_dashboard)
- `/cmd/*` — forwarded commands (vehicle_control_node → bridge)
- `/vehicle/state` — vehicle telemetry (bridge → all displays)
- `/vehicle/imu`, `/vehicle/gps` — sensor data (pixhawk_bridge only)

## Deployment

Mac runs polaris_wheel (standalone binary), connecting directly to fastgpu over WiFi via QUIC. Everything else runs on fastgpu. One script (`launch/sim.sh`) handles sync, build, SITL, and all nodes.
