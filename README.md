# Polaris

Remote-operated Polaris side-by-side over ROS2. Drive from a Mac with a G923 racing wheel — commands flow over QUIC to fastgpu, which runs all ROS2 nodes and bridges to a Pixhawk autopilot (real or SITL).

## Prerequisites

- ROS2 Jazzy (on fastgpu)
- Rust 1.80+
- ArduPilot SITL + mavproxy for simulation (Python 3.12 venv on fastgpu)
- `f` SSH alias pointing to fastgpu (Arch Linux build machine)
- SDL2 (on Mac, for polaris_wheel)

## Build

```bash
# Wheel app builds locally on Mac
cargo build --release -p polaris_wheel

# Everything else builds on fastgpu via sync
bash sync.sh --build
```

The `sync.sh` script rsyncs the workspace to fastgpu and runs `cargo build --release` remotely. polaris_msgs must be pre-built with colcon on fastgpu:

```bash
# One-time setup on fastgpu
cd ~/ros2_ws/src && ln -sf ~/src/experimental/polaris/polaris_msgs .
cd ~/ros2_ws && colcon build --packages-select polaris_msgs
```

## Quick Start -- Simulation (SITL)

```bash
./launch/sim.sh                    # syncs, builds, launches SITL, opens TUI
./launch/sim.sh --skip-sitl        # reconnect TUI to already-running rig
```

Once the TUI is open, drive with WASD (keyboard) or connect the G923:

```bash
# On Mac — connect physical wheel to running rig
./target/release/polaris_wheel fastgpu.local:9876
```

## Architecture

```
Mac                                     fastgpu (all ROS2 nodes)
────                                    ────────────────────────
G923 → polaris_wheel ──QUIC:9876──→ wheel_bridge_node
                                       │ /input/*
                                       ▼
                                  vehicle_control_node
                                       │ /cmd/*
                                       ▼
                                  pixhawk_bridge_node ←→ Pixhawk / SITL
                                       │
                                       ▼
                                  /vehicle/state, /vehicle/imu, /vehicle/gps
                                       │
            polaris_wheel ←──QUIC──── wheel_bridge_node
            tui_dashboard (SSH) ←── /vehicle/*
```

Six Rust crates, one ROS2 message package:

| Crate | Binary | Role |
|-------|--------|------|
| polaris_msgs | -- | VehicleState.msg (colcon build) |
| polaris_wheel | `polaris_wheel` | G923 wheel app on Mac (SDL2 + QUIC client) |
| polaris_operator | `tui_dashboard`, `gamepad_node`, `wheel_bridge_node` | Operator TUI + input bridges |
| polaris_control | `vehicle_control_node` | Passthrough: /input/* → /cmd/* at 50Hz |
| polaris_bridge | `pixhawk_bridge_node` | MAVLink to Pixhawk (real + SITL) |
| polaris_sim | `beamng_bridge_node` | BeamNG MessagePack bridge |
