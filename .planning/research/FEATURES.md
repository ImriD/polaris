# FEATURES — Jetson + Pixhawk Hardware Table Rig

**Date:** 2026-02-23
**Milestone:** Table rig for Polaris vehicle control (Jetson NRU-160 + Pixhawk over Ethernet)
**Question:** What features does a Jetson + Pixhawk bench test rig need? What's table stakes vs differentiating?

---

## Context

The Polaris system already exists: G923 wheel on Mac → QUIC → ROS2 nodes → MAVLink → Pixhawk.
This rig moves the ROS2 + MAVLink portion from a simulation server (fastgpu, x86_64) to a physical
Jetson NRU-160 (aarch64) connected to a real Pixhawk over Ethernet through a Teltonika router.

Goal: validate the full command pipeline with real Pixhawk on a table before the rig goes on the vehicle.

Network: Mac at 192.168.88.108, Jetson at 192.168.88.175, Pixhawk at 192.168.88.242.

---

## Table Stakes
_Must have. Without these the rig doesn't serve its purpose._

### 1. End-to-End Command Pipe Works
**What:** G923 wheel on Mac → QUIC → wheel_bridge_node on Jetson → /cmd/* → pixhawk_bridge_node → RC_CHANNELS_OVERRIDE → real Pixhawk acknowledges.
**Why:** This is the entire point of the rig. Everything else is either scaffolding to reach this or telemetry to observe it.
**Complexity:** Medium. All three pieces exist; what's new is stitching them on aarch64 with a network Pixhawk address.
**Dependencies:** Jetson provisioned (feature 2), Pixhawk reachable over Ethernet (feature 3), QUIC reachable on Teltonika network (feature 4).

### 2. Jetson Running ROS2 Humble + Polaris Crates
**What:** ROS2 Humble installed, Rust toolchain installed, all polaris crates compile and run on aarch64 Ubuntu 22.04.
**Why:** Nothing else can happen without this. The Jetson is fresh — no ROS2, no Rust yet.
**Complexity:** Low-medium. Native compilation on Jetson avoids cross-compilation fragility. r2r is compatible with both Humble and Jazzy. Main risk is colcon build for polaris_msgs on aarch64.
**Dependencies:** None (this is the foundation). fastgpu setup must stay untouched — these are additive changes.

### 3. Pixhawk Reachable Over Ethernet MAVLink
**What:** pixhawk_bridge_node connects to Pixhawk at 192.168.88.242 via UDP (or TCP) MAVLink and exchanges heartbeats.
**Why:** Current code targets serial/TCP-loopback (SITL). Ethernet address is a new target. Without confirmed heartbeat exchange, nothing downstream is reliable.
**Complexity:** Low. The mavlink crate supports UDP and TCP out of the box. The change is a new PIXHAWK_ADDR env var value (e.g. `udpout:192.168.88.242:14550`). Main risk: finding the correct port the Pixhawk listens on, and whether it requires a UDP client that sends first.
**Dependencies:** Jetson provisioned (feature 2), Pixhawk on the same L2 network.

### 4. QUIC Reachable from Mac to Jetson
**What:** polaris_wheel on Mac can reach wheel_bridge_node on Jetson at 192.168.88.175:9876. The existing QUIC TLS cert workflow must work on the Jetson.
**Why:** Operator input cannot reach the Jetson without this. The network is a new environment (Teltonika, different from WiFi to fastgpu).
**Complexity:** Low. QUIC uses UDP; Teltonika routers forward UDP by default. Cert generation on Jetson is a one-time step. Main risk: firewall rules on Jetson or port conflicts.
**Dependencies:** Jetson provisioned (feature 2).

### 5. Telemetry Returns to Operator
**What:** VehicleState (speed, heading, IMU, position) published by pixhawk_bridge_node, serialized by wheel_bridge_node, and received by polaris_wheel TUI display on Mac.
**Why:** Without telemetry the operator is flying blind and cannot verify commands landed. Also required to detect Pixhawk connection loss.
**Complexity:** Low — same 72-byte QUIC downlink protocol already exists for SITL. The Pixhawk will return GLOBAL_POSITION_INT and RAW_IMU over MAVLink once connected.
**Dependencies:** Feature 3 (MAVLink connection), Feature 4 (QUIC return path).

### 6. Launch Script for the Table Rig
**What:** A single script (e.g. `launch/rig.sh`) that syncs code to Jetson, builds on Jetson, and starts all nodes (wheel_bridge_node, vehicle_control_node, pixhawk_bridge_node) with the right env vars.
**Why:** Without a repeatable launch, every test session is manual and error-prone. The existing `launch/sim.sh` is the model.
**Complexity:** Low. Mostly an adaptation of `launch/sim.sh` with different host, env vars, and no SITL steps.
**Dependencies:** Features 2–4 (build, Pixhawk address, QUIC port). Must not touch or break `launch/sim.sh`.

---

## Differentiators
_Nice to have. These make the rig more useful but the core validation works without them._

### 7. Pixhawk Armed + Mode Confirmed in TUI
**What:** Display Pixhawk HEARTBEAT data (arm state, flight mode) in the tui_dashboard or polaris_wheel display alongside actuator bars.
**Why:** Currently mode and arm state come back as opaque integers in VehicleState. Bench testing benefits from seeing "MANUAL / DISARMED" explicitly — confirms the Pixhawk is in the expected state before commands matter.
**Complexity:** Low. Heartbeat mode/arm is already read by PixhawkInterface; it's a display change.
**Dependencies:** Feature 3, Feature 5.

### 8. ros2 bag Recording During Test Sessions
**What:** Capture all /input/*, /cmd/*, /vehicle/* topics to a bag file on the Jetson during runs.
**Why:** Allows post-session replay and debugging. If something goes wrong, you can replay exactly what the Pixhawk received. Especially useful when diagnosing latency or command drops.
**Complexity:** Low. `ros2 bag record -a` works out of the box. Consider MCAP format over SQLite3 for better performance at 50Hz.
**Dependencies:** Feature 2 (ROS2 on Jetson).
**Note:** Storage is abundant (202GB free). Bags are gitignored.

### 9. End-to-End Latency Measurement
**What:** Timestamp the QUIC packet at send (Mac), at receipt (Jetson), and at MAVLink dispatch; log the deltas.
**Why:** One key question this rig answers is: does 50Hz control over QUIC + MAVLink over Ethernet fit within an acceptable latency budget for real driving? Measurement makes this concrete.
**Complexity:** Medium. Requires synchronized clocks (NTP or PTP between Mac and Jetson) or relative timestamps with known drift. Can be approximated with log timestamps.
**Dependencies:** Feature 1 (full pipeline), Feature 6 (repeatable launch).

### 10. Motor/Actuator Response Verification
**What:** Confirm that Pixhawk RC_CHANNELS_OVERRIDE values actually change actuator outputs — observe ESC signals or servo positions change when wheel input changes.
**Why:** MAVLink accept ≠ actuator moves. The rig should verify the full chain down to physical actuation, not just protocol acknowledgment.
**Complexity:** Medium. Requires either QGroundControl motor test, an oscilloscope on PWM output lines, or attaching a servo/ESC to the bench rig and observing movement.
**Dependencies:** Feature 1. Physical setup (propellers off, ESC on bench, safe environment).

---

## Anti-Features
_Things to deliberately not build for this milestone._

### Do Not: Autonomous / Offboard Control Mode
The rig is manual-control only. Adding PX4 offboard mode or waypoint following introduces autonomous safety requirements (geofencing, failsafe modes, kill switch hardware) that are out of scope for a bench validation pass. Adding this now adds complexity without validating the core question.

### Do Not: Cross-Compile for aarch64 from Mac or fastgpu
Cross-compiling Rust + r2r + colcon for aarch64 is fragile — library paths, colcon overlays, and ROS2 ABI compatibility all become moving parts. The Jetson has 16GB RAM and 202GB NVMe; native compilation is slower but reliable. Cross-compilation can be added later when iteration speed matters.

### Do Not: Modify fastgpu / sim Setup
The simulation workflow on fastgpu must remain untouched. All table rig changes are additive: new env var values, a new launch script, new Jetson-specific provisioning. If fastgpu breaks, it is not a rig problem.

### Do Not: GPS / Outdoor Testing
The table rig is indoor, bench only. GPS requires spoofing or outdoor environment — neither is appropriate here. polaris_bridge should gracefully handle absent GPS without crashing.

### Do Not: Hardware E-Stop / Safety Interlock Hardware
A proper E-stop (physical button, hardwired to kill power) is a deployment requirement, not a bench testing requirement. For bench use, "table rig" implies propellers off and controlled environment. Adding E-stop hardware now scopes-creeps a validation milestone into a safety certification task.

### Do Not: Dashboard / Web UI for Remote Monitoring
The existing TUI (ratatui) and polaris_wheel display are sufficient for bench observation. A browser-based dashboard is a nice-to-have for the vehicle deployment phase, not the table validation phase.

### Do Not: BeamNG Bridge on Jetson
BeamNG simulation stays on fastgpu. The Jetson is for real hardware. Running both introduces complexity and splits the deployment target.

---

## Dependency Map

```
Feature 2 (Jetson provisioned)
  └─► Feature 3 (Pixhawk MAVLink)
  └─► Feature 4 (QUIC reachable)
        └─► Feature 5 (Telemetry)
              └─► Feature 7 (arm/mode in TUI)  [differentiator]
  └─► Feature 6 (Launch script)
Feature 3 + Feature 4 + Feature 5
  └─► Feature 1 (End-to-end pipe)
        └─► Feature 9 (Latency measurement)    [differentiator]
        └─► Feature 10 (Actuator response)     [differentiator]
Feature 2 alone
  └─► Feature 8 (ros2 bag recording)           [differentiator]
```

---

## Summary Table

| # | Feature | Category | Complexity |
|---|---------|----------|------------|
| 1 | End-to-end command pipe | Table stakes | Medium |
| 2 | Jetson: ROS2 Humble + Rust + polaris crates | Table stakes | Low-Med |
| 3 | Pixhawk over Ethernet MAVLink | Table stakes | Low |
| 4 | QUIC reachable Mac → Jetson | Table stakes | Low |
| 5 | Telemetry returns to operator | Table stakes | Low |
| 6 | Launch script for table rig | Table stakes | Low |
| 7 | Arm/mode displayed in TUI | Differentiator | Low |
| 8 | ros2 bag recording | Differentiator | Low |
| 9 | End-to-end latency measurement | Differentiator | Medium |
| 10 | Actuator response verification | Differentiator | Medium |
| — | Autonomous/offboard mode | Anti-feature | — |
| — | Cross-compile from Mac/fastgpu | Anti-feature | — |
| — | Modify fastgpu/sim setup | Anti-feature | — |
| — | GPS/outdoor testing | Anti-feature | — |
| — | Hardware E-stop | Anti-feature | — |
| — | Web dashboard | Anti-feature | — |
| — | BeamNG on Jetson | Anti-feature | — |

---

## Key Insight

The table stakes are entirely about making the existing code run in a new environment (aarch64, Ethernet Pixhawk, Teltonika network). Nothing new needs to be written for the core validation — just provisioning, configuration, and a launch script. If feature 1 passes, the milestone is done.

The differentiators (7–10) are high-value additions that cost little effort but significantly increase confidence that the rig is production-ready. Feature 8 (bag recording) in particular is almost free and provides replay capability that pays off in every future debug session.

---

Sources:
- [Holybro Pixhawk Jetson Baseboard | PX4 Guide](https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard)
- [ROS2 Offboard on Hardware — PX4 Discussion Forum](https://discuss.px4.io/t/ros2-offboard-on-hardware/26338)
- [Design Your Robot on Hardware-in-the-Loop with NVIDIA Jetson | NVIDIA Technical Blog](https://developer.nvidia.com/blog/design-your-robot-on-hardware-in-the-loop-with-nvidia-jetson/)
- [Multi-Mode MAVLink Test Bench AeroQT — Frontiers](https://www.frontierspartnerships.org/journals/aerospace-research-communications/articles/10.3389/arc.2025.14524/full)
- [Hardware-in-the-Loop (HIL) Test System Architectures — NI](https://www.ni.com/en/solutions/transportation/hardware-in-the-loop/hardware-in-the-loop--hil--test-system-architectures.html)
- [Record and Play Back Data Using ROS 2 Bag — Automatic Addison](https://automaticaddison.com/record-and-play-back-data-using-ros-2-bag-ros-2-jazzy/)
- [ROS 2-Based Architecture for Autonomous Driving Systems — MDPI Sensors](https://www.mdpi.com/1424-8220/26/2/463)
- [Even Lower Latency in IIoT: Evaluation of QUIC — PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC8434189/)
