# Polaris Table Rig

## What This Is

A hardware test rig deploying the Polaris ROS2 vehicle control stack onto a Jetson NRU-160, connected to a real Pixhawk over MAVLink/Ethernet through a Teltonika RUTM51 router. Operator drives from a Mac with a G923 racing wheel over QUIC. The existing fastgpu simulation setup remains untouched.

## Core Value

Commands from the Mac reach the real Pixhawk through the Jetson — the full hardware path works end-to-end on the table before going on the vehicle.

## Requirements

### Validated

- steering/throttle/brake/gear/handbrake command pipeline (existing code)
- QUIC transport between Mac and ROS2 bridge node (existing code)
- MAVLink RC_CHANNELS_OVERRIDE to Pixhawk (existing code)
- VehicleState telemetry back to operator (existing code)
- TUI dashboard for monitoring (existing code)

### Active

- [ ] Jetson provisioned with ROS2 Humble, Rust toolchain, and all build dependencies
- [ ] Polaris crates cross-compile and run on Jetson (aarch64, Ubuntu 22.04)
- [ ] pixhawk_bridge_node connects to Pixhawk over Ethernet MAVLink at 192.168.88.242
- [ ] wheel_bridge_node accepts QUIC connections from Mac on the Teltonika network
- [ ] Launch script for the table rig (sync, build, start all nodes on Jetson)
- [ ] End-to-end verified: G923 wheel on Mac → QUIC → Jetson → MAVLink → Pixhawk responds

### Out of Scope

- Modifying the fastgpu/sim setup — stays as-is
- Autonomous mode — table rig is manual control only
- BeamNG bridge on the Jetson — sim stays on fastgpu
- GPS/outdoor testing — table rig is bench testing

## Context

**Network topology:**
- Teltonika RUTM51 router at 192.168.88.1
- Mac (operator) at 192.168.88.108 via USB Ethernet adapter (en7)
- Jetson NRU-160 (tegra-ubuntu) at 192.168.88.175
- Pixhawk (nuttx) at 192.168.88.242 — Ethernet-native MAVLink

**Jetson state (as of 2026-02-23):**
- Ubuntu 22.04 Jammy, aarch64, Linux 5.15.136-tegra
- 16GB RAM, 233GB NVMe (202GB free)
- No ROS2, no Rust — fresh machine
- SSH: nvidia@192.168.88.175 (password: nvidia)

**Existing codebase:**
- 6 Rust crates, all currently targeting fastgpu (Arch Linux x86_64, ROS2 Jazzy)
- r2r bindings work with both Humble and Jazzy
- Pixhawk bridge currently connects via serial or TCP loopback (SITL) — needs to target Ethernet address

**Key difference from fastgpu:**
- Ubuntu 22.04 → ROS2 Humble (not Jazzy)
- aarch64 (not x86_64) — native compilation on-device
- Pixhawk over Ethernet at a network address (not serial/localhost)

## Constraints

- **ROS2 version**: Humble (Ubuntu 22.04 only supports Humble, not Jazzy)
- **Architecture**: aarch64 — must compile natively on the Jetson
- **Network**: All devices on 192.168.88.x via Teltonika; no internet assumed during operation
- **Pixhawk protocol**: MAVLink over UDP/TCP to 192.168.88.242 (not serial)

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| ROS2 Humble on Jetson | Ubuntu 22.04 doesn't support Jazzy; r2r works with both | -- Pending |
| Native compilation on Jetson | Cross-compilation for aarch64 ROS2 is fragile; Jetson has plenty of resources | -- Pending |
| Keep fastgpu setup untouched | Table rig is additive — sim workflow must keep working | -- Pending |
| MAVLink over Ethernet | Pixhawk has native Ethernet, shows up at 192.168.88.242 | -- Pending |

---
*Last updated: 2026-02-23 after initialization*
