# Roadmap: Polaris Table Rig

## Overview

Provision a fresh Jetson NRU-160, connect it to a real Pixhawk over Ethernet MAVLink, link the Mac operator over QUIC, and validate the full command pipeline end-to-end. The existing fastgpu simulation setup is untouched throughout. Four sequential phases, each unblocking the next.

## Phases

- [ ] **Phase 1: Jetson Provisioning** - Jetson can build and run all polaris crates natively on aarch64
- [ ] **Phase 2: Pixhawk MAVLink** - pixhawk_bridge_node exchanges heartbeat and streams telemetry from real Pixhawk
- [ ] **Phase 3: QUIC + Launch** - polaris_wheel connects from Mac; launch/rig.sh brings the full stack up in one command
- [ ] **Phase 4: End-to-End Validation** - G923 wheel commands reach Pixhawk and telemetry returns to Mac TUI

## Phase Details

### Phase 1: Jetson Provisioning
**Goal**: Jetson can build and run all polaris crates natively on aarch64
**Depends on**: Nothing (first phase)
**Requirements**: PROV-01, PROV-02, PROV-03, PROV-04, PROV-05
**Success Criteria** (what must be TRUE):
  1. `ros2 topic list` runs without error on the Jetson
  2. `rustc --version` on Jetson reports 1.75+ (rustup-installed, not apt)
  3. `cargo build --release` for polaris_bridge completes on Jetson with ROS2 env sourced
  4. `ros2 topic echo /input/steering` receives messages from a manually run polaris node
**Plans**: TBD

### Phase 2: Pixhawk MAVLink
**Goal**: pixhawk_bridge_node exchanges heartbeat and streams telemetry from real Pixhawk over Ethernet
**Depends on**: Phase 1
**Requirements**: MAVL-01, MAVL-02, MAVL-03
**Success Criteria** (what must be TRUE):
  1. `tcpdump -i any udp port 14550` on Jetson shows UDP packets arriving from 192.168.88.242
  2. pixhawk_bridge_node logs a heartbeat received from the Pixhawk within 5 seconds of start
  3. `ros2 topic hz /vehicle/state` on Jetson shows messages arriving at ~10Hz
**Plans**: TBD

### Phase 3: QUIC + Launch
**Goal**: Mac connects to Jetson QUIC server and launch/rig.sh starts the full stack reliably
**Depends on**: Phase 2
**Requirements**: QUIC-01, QUIC-02, QUIC-03, LNCH-01, LNCH-02, LNCH-03
**Success Criteria** (what must be TRUE):
  1. `./launch/rig.sh` on Mac syncs code, builds on Jetson, and starts all nodes without manual SSH steps
  2. polaris_wheel on Mac connects to 192.168.88.175:9876 and the TUI renders without TLS errors
  3. `ros2 topic hz /input/steering` on Jetson shows live input arriving from the Mac
  4. Restarting all nodes via `launch/rig.sh` succeeds without regenerating the TLS cert or losing the QUIC connection
**Plans**: TBD

### Phase 4: End-to-End Validation
**Goal**: G923 wheel commands reach the Pixhawk and VehicleState telemetry returns to the operator TUI
**Depends on**: Phase 3
**Requirements**: VALD-01, VALD-02
**Success Criteria** (what must be TRUE):
  1. Turning the G923 wheel produces RC_CHANNELS_OVERRIDE messages visible on Pixhawk (via Mission Planner or mavproxy status)
  2. The polaris_wheel TUI on Mac displays live VehicleState values that change when the Pixhawk moves
**Plans**: TBD

## Progress

**Execution Order:** 1 → 2 → 3 → 4

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Jetson Provisioning | 0/TBD | Not started | - |
| 2. Pixhawk MAVLink | 0/TBD | Not started | - |
| 3. QUIC + Launch | 0/TBD | Not started | - |
| 4. End-to-End Validation | 0/TBD | Not started | - |
