# Requirements: Polaris Table Rig

**Defined:** 2026-02-23
**Core Value:** Commands from the Mac reach the real Pixhawk through the Jetson — the full hardware path works end-to-end on the table

## v1 Requirements

### Provisioning

- [ ] **PROV-01**: Jetson has ROS2 Humble installed (`ros-humble-ros-base` from apt)
- [ ] **PROV-02**: Jetson has Rust toolchain installed via rustup (aarch64-native)
- [ ] **PROV-03**: Jetson has build dependencies installed (libclang-dev, cmake, pkg-config)
- [ ] **PROV-04**: polaris_msgs built in colcon workspace on Jetson (`~/ros2_ws/`)
- [ ] **PROV-05**: All polaris crates compile on Jetson (polaris_bridge, polaris_operator, polaris_control)

### MAVLink Ethernet

- [ ] **MAVL-01**: pixhawk_bridge_node connects to Pixhawk at 192.168.88.242 over UDP MAVLink
- [ ] **MAVL-02**: Pixhawk NET_P1 parameters configured for Ethernet MAVLink
- [ ] **MAVL-03**: Telemetry streams (GLOBAL_POSITION_INT, RAW_IMU) flow from Pixhawk to bridge node

### QUIC Transport

- [ ] **QUIC-01**: wheel_bridge_node listens on 0.0.0.0:9876 on the Jetson
- [ ] **QUIC-02**: polaris_wheel on Mac connects to 192.168.88.175:9876 over Teltonika network
- [ ] **QUIC-03**: TLS cert generated with correct Jetson IP SAN, persisted to disk

### Launch

- [ ] **LNCH-01**: `launch/rig.sh` syncs code, builds, and starts all nodes on Jetson
- [ ] **LNCH-02**: ROS2 env vars preserved through nohup (LD_LIBRARY_PATH forwarded)
- [ ] **LNCH-03**: DDS confined to Jetson localhost (`ROS_LOCALHOST_ONLY=1`)

### Validation

- [ ] **VALD-01**: End-to-end verified: G923 wheel commands reach Pixhawk as RC_CHANNELS_OVERRIDE
- [ ] **VALD-02**: Telemetry (VehicleState) returns to polaris_wheel TUI on Mac

## v2 Requirements

### Observability

- **OBSV-01**: ros2 bag recording in MCAP format during test sessions
- **OBSV-02**: End-to-end latency measurement through full pipeline
- **OBSV-03**: Pixhawk arm/mode state displayed in TUI

## Out of Scope

| Feature | Reason |
|---------|--------|
| Autonomous/offboard mode | Safety requirements explode scope |
| Cross-compilation from Mac | Fragile with r2r bindgen; Jetson has 16GB RAM for native builds |
| BeamNG on Jetson | Sim stays on fastgpu |
| GPS/outdoor testing | Bench testing only |
| Modifying fastgpu setup | Simulation workflow must remain working |
| Hardware E-stop | Vehicle deployment concern, not bench validation |
| Web dashboard | TUI is sufficient for bench observation |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| PROV-01 | - | Pending |
| PROV-02 | - | Pending |
| PROV-03 | - | Pending |
| PROV-04 | - | Pending |
| PROV-05 | - | Pending |
| MAVL-01 | - | Pending |
| MAVL-02 | - | Pending |
| MAVL-03 | - | Pending |
| QUIC-01 | - | Pending |
| QUIC-02 | - | Pending |
| QUIC-03 | - | Pending |
| LNCH-01 | - | Pending |
| LNCH-02 | - | Pending |
| LNCH-03 | - | Pending |
| VALD-01 | - | Pending |
| VALD-02 | - | Pending |

**Coverage:**
- v1 requirements: 16 total
- Mapped to phases: 0
- Unmapped: 16

---
*Requirements defined: 2026-02-23*
*Last updated: 2026-02-23 after initial definition*
