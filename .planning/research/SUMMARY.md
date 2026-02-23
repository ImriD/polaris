# Project Research Summary

**Project:** Polaris — Jetson + Pixhawk Hardware Table Rig
**Domain:** Embedded robotics / ROS2 hardware deployment
**Researched:** 2026-02-23
**Confidence:** HIGH

## Executive Summary

Polaris already has a working control pipeline: G923 wheel on Mac sends commands over QUIC to ROS2 nodes on a Linux server, which translates them to MAVLink and drives a Pixhawk. This milestone moves the ROS2 + MAVLink portion from a simulation server (fastgpu, x86_64, Jazzy) to a physical Jetson NRU-160 (aarch64, Humble) connected to a real Pixhawk over Ethernet through a Teltonika router. The research confirms the existing code requires almost no changes — all adaptation is provisioning, env vars, and a new launch script.

The recommended approach is native compilation on the Jetson (not cross-compile), ROS2 Humble from apt (not Jazzy, not source build), and MAVLink over UDP with the Pixhawk acting as the UDP client reaching out to the Jetson. The single most important first step is provisioning the Jetson correctly (ROS2, Rust, libclang, polaris_msgs colcon workspace) — everything else depends on that foundation being solid.

The critical risks are operational rather than architectural: nohup dropping ROS2 environment variables is the most likely first-session blocker, followed by a UDP client/server mismatch between the Pixhawk and bridge (which requires a Pixhawk reboot to change after setting parameters). The architecture is sound and battle-tested; the pitfalls are all known and preventable with explicit launch script discipline.

## Key Findings

### Recommended Stack

The existing Rust + r2r + ROS2 stack ports cleanly to aarch64. ROS2 Humble is the correct choice for Ubuntu 22.04 Jammy — it has official binary apt packages for arm64 and its EOL (May 2027) comfortably exceeds this milestone's horizon. Jazzy would require a source build on 22.04 and is not worth the pain. The Rust toolchain must come from rustup (not apt) because Ubuntu 22.04's packaged Rust 1.66 is too old; the project requires recent stable. Cross-compilation from Mac or fastgpu is explicitly rejected — r2r's bindgen step requires libclang to introspect the target's ROS2 headers, making cross-compile fragile when native compile on the Jetson takes only 5-10 minutes.

**Core technologies:**
- `ros-humble-ros-base`: ROS2 middleware for aarch64 — only correct choice on Ubuntu 22.04; saves 500MB vs desktop
- `rustup stable (≥1.93.0)`: Rust toolchain — apt rustc is too old (1.66); rustup installs current stable
- `r2r 0.9.5` (unchanged): Rust ROS2 bindings — Humble/Jazzy compatible, no code changes needed
- `mavlink 0.14.1` (unchanged): MAVLink crate — `udpin:0.0.0.0:14550` for Ethernet connection, no code changes
- `quinn 0.11.9` (unchanged): QUIC transport — platform-agnostic, runs identically on aarch64
- `libclang-dev` + `cmake` + `pkg-config`: r2r build deps — bindgen hard requirement at compile time
- `IDL_PACKAGE_FILTER` in `.cargo/config.toml`: limits r2r message binding generation — already configured, critical for build time

### Expected Features

The six table-stakes features all exist in the current codebase; this milestone is about porting, not building. The dependency chain is strict: Jetson provisioning must succeed before anything else can be tested.

**Must have (table stakes):**
- Jetson running ROS2 Humble + Rust + polaris crates — foundation; nothing else works without it
- Pixhawk reachable over Ethernet MAVLink — new address, same mavlink crate, env var change only
- QUIC reachable Mac to Jetson — same quinn, network is same L2 LAN, cert must include Jetson IP SAN
- Telemetry returns to operator — same 72-byte downlink protocol, Pixhawk stream rates need explicit config
- End-to-end command pipe works — the milestone validation test; if this passes, the rig is done
- Launch script `launch/rig.sh` — repeatable test session entry point; mirrors `launch/sim.sh`

**Should have (differentiators):**
- Arm/mode displayed in TUI — low effort, confirms Pixhawk state before commanding
- `ros2 bag record -a` during test sessions — nearly free, provides replay for any future debug session
- End-to-end latency measurement — validates that 50Hz over QUIC + Ethernet MAVLink fits the control budget
- Motor/actuator response verification — confirms RC_CHANNELS_OVERRIDE actually moves hardware

**Defer (out of scope for this milestone):**
- Autonomous/offboard control — introduces safety requirements inappropriate for a bench validation pass
- Cross-compilation from Mac/fastgpu — native compile is reliable; cross-compile is fragile for r2r
- Any modification to the fastgpu/sim workflow — all rig changes are strictly additive
- GPS/outdoor testing — indoor bench only; polaris_bridge should handle absent GPS gracefully
- Hardware E-stop or web dashboard — deployment concerns, not bench validation concerns

### Architecture Approach

The architecture is clean and already dual-target ready. All ROS2 pub/sub stays local to the Jetson — DDS never spans the Teltonika network. The Mac connects only via QUIC on port 9876. fastgpu keeps its isolated ROS2 domain and is untouched. The only adaptation surface is: `PIXHAWK_ADDR` env var, `POLARIS_SERVER` env var (or CLI arg) in `polaris_wheel`, and a new `launch/rig.sh`. No source code changes are required for the core pipeline.

**Major components:**
1. `polaris_wheel` (Mac) — SDL2 reads G923 wheel, sends 14-byte commands at 50Hz over QUIC, renders TUI
2. `wheel_bridge_node` (Jetson) — QUIC server port 9876, bridges QUIC packets to ROS2 `/input/*` topics
3. `vehicle_control_node` (Jetson) — normalizes input, re-publishes `/cmd/*` at exactly 50Hz wall-timer
4. `pixhawk_bridge_node` (Jetson) — subscribes `/cmd/*`, sends RC_CHANNELS_OVERRIDE to Pixhawk over Ethernet MAVLink, publishes `/vehicle/*` telemetry
5. `tui_dashboard` (Jetson, SSH session) — subscribes all topics, renders live Hz rates and actuator bars

### Critical Pitfalls

1. **nohup drops ROS2 env vars (P1)** — Pass `LD_LIBRARY_PATH=$LD_LIBRARY_PATH` explicitly in every nohup command and source both `setup.bash` files in the same SSH session that launches the binaries. This is already known from fastgpu; the Humble path (`/opt/ros/humble/`) differs from Jazzy.

2. **Pixhawk UDP client/server mismatch (P3)** — Establish convention at setup time: Pixhawk as UDP client (`NET_P1_TYPE=1`, destination `192.168.88.175:14550`), bridge as `udpin:0.0.0.0:14550`. Reboot Pixhawk after every `NET_P*` change. Verify with `tcpdump -i any udp port 14550` before starting bridge.

3. **QUIC TLS cert IP mismatch (P7)** — Generate cert once with Jetson's LAN IP (`192.168.88.175`) as SAN. Persist to disk. Do not regenerate on restart. Bundle public cert with `polaris_wheel` or load via env var.

4. **r2r links wrong ROS2 install (P6)** — Always build with both `setup.bash` files sourced. Verify `$AMENT_PREFIX_PATH` includes `~/ros2_ws/install/polaris_msgs`. Never run bare `cargo build` without the ROS2 environment.

5. **Pixhawk stream rates not configured for Ethernet port (P9)** — After heartbeat confirmed, set `MAV_2_MODE=2` (onboard) and `MAV_2_RATE=10` on the Pixhawk. Without this, `/vehicle/*` topics will be empty despite a successful MAVLink connection.

## Implications for Roadmap

Based on the dependency graph in FEATURES.md and the pitfall sequence from PITFALLS.md, the natural phase structure is: provision first, then integrate each network boundary in order of physical distance from the operator, then validate end-to-end.

### Phase 1: Jetson Provisioning
**Rationale:** Everything downstream depends on this. No Rust build, no ROS2 node, no test is possible until the Jetson has ROS2 Humble, Rust, libclang, and a built polaris_msgs colcon workspace. This is also where P8 (JetPack OpenCV conflicts) and P4 (IDL_PACKAGE_FILTER) must be handled — fail here and all subsequent phases are blocked.
**Delivers:** Jetson can build and run all polaris crates natively on aarch64.
**Addresses:** Feature 2 (Jetson: ROS2 Humble + Rust + polaris crates).
**Avoids:** P4 (unbounded r2r build time), P6 (wrong ROS2 install linked), P8 (OpenCV conflicts blocking apt).

### Phase 2: Pixhawk Ethernet MAVLink Integration
**Rationale:** The Pixhawk connection is the most uncertain element — it requires hardware configuration (NET_P1 parameters, reboot), the correct UDP client/server convention, and explicit stream rate setup. These are sequential steps that cannot be tested without a connected Pixhawk. Isolate this phase before adding QUIC complexity.
**Delivers:** `pixhawk_bridge_node` exchanges heartbeat and streams telemetry from real Pixhawk. `/vehicle/*` topics publish at 10Hz.
**Addresses:** Feature 3 (Pixhawk over Ethernet MAVLink), Feature 5 (Telemetry returns to operator, partially).
**Avoids:** P3 (UDP client/server mismatch), P9 (stream rates not configured).

### Phase 3: QUIC Integration and Launch Script
**Rationale:** With the Jetson building and the Pixhawk connected, the final integration point is the QUIC link from Mac to Jetson. This includes TLS cert setup with the correct IP SAN, verifying the Teltonika network forwards UDP, and writing `launch/rig.sh` as the repeatable entry point for all future sessions.
**Delivers:** `polaris_wheel` on Mac connects to Jetson QUIC server. Full pipeline is up. `launch/rig.sh` brings everything up in one command.
**Addresses:** Feature 4 (QUIC reachable Mac to Jetson), Feature 6 (Launch script).
**Avoids:** P1 (nohup env vars — handled in launch script), P5 (DDS multicast — set ROS_LOCALHOST_ONLY=1 in script), P7 (TLS cert IP mismatch).

### Phase 4: End-to-End Validation
**Rationale:** With the pipeline up, validate that G923 wheel input reaches the Pixhawk and produces a measurable response. This is the milestone acceptance test. Differentiator features (bag recording, arm/mode in TUI, latency measurement, actuator response) are natural additions here — they all depend on the full pipeline and cost little effort.
**Delivers:** Confirmed end-to-end command pipe. Optional: bag recording, latency numbers, actuator observation.
**Addresses:** Feature 1 (End-to-end command pipe), Features 7-10 (differentiators).

### Phase Ordering Rationale

- Provisioning before integration because r2r bindgen (P6) and build dependencies (P4, P8) must be settled before any node can run.
- Pixhawk before QUIC because the Pixhawk has more unknowns (hardware config, reboot requirements, parameter persistence) and its failure mode (no heartbeat) is unambiguous. QUIC failures are subtler (TLS errors) and easier to debug once the Pixhawk path is confirmed working.
- Launch script authored in Phase 3 (not earlier) because it encodes all the env vars and sourcing patterns learned in Phases 1-2. Writing it first would require rework.
- Validation last because it depends on all prior phases and its differentiator features are most useful once the core pipeline is stable.

### Research Flags

Phases with standard patterns (research-phase not needed):
- **Phase 1 (Provisioning):** Well-documented; official ROS2 Humble aarch64 install docs are authoritative. rustup install is trivial. r2r colcon workflow already tested on fastgpu.
- **Phase 3 (QUIC/Launch):** Existing `launch/sim.sh` is the direct template. quinn cert pattern is documented. Teltonika UDP forwarding is default behavior.
- **Phase 4 (Validation):** No new code; this is observation and testing.

Phase that may benefit from targeted investigation before planning:
- **Phase 2 (Pixhawk MAVLink):** The specific Pixhawk variant in the NRU-160 rig and its exact Ethernet parameter defaults are unknown. The ArduPilot NET_P1 framework documentation covers the general case but the NRU-160's specific hardware may have different port defaults or parameter naming. Recommend verifying with `mavproxy.py --master=udpin:0.0.0.0:14550` live before writing the phase plan.

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | All components verified against official docs; identical stack already running on fastgpu (Jazzy variant) |
| Features | HIGH | Feature set is minimal and derived from the existing codebase; no new user-facing requirements |
| Architecture | HIGH | Architecture is unchanged from working fastgpu setup; adaptation surface is small and explicit |
| Pitfalls | HIGH | P1 and P6 are already known-encountered pitfalls from fastgpu; remaining pitfalls sourced from ArduPilot and ROS2 community with multiple confirming sources |

**Overall confidence:** HIGH

### Gaps to Address

- **Pixhawk exact model and NET_P1 defaults:** The NRU-160 rig's Pixhawk variant is not specified in research. The correct `NET_P1_TYPE` starting value and whether a first-boot Ethernet config is already set should be verified physically before Phase 2 begins. Use `mavproxy.py` to probe what the Pixhawk is broadcasting before configuring the bridge.
- **`polaris_wheel` server address hardcoding:** Architecture research notes that `polaris_wheel` currently has the fastgpu address hardcoded. This needs a CLI arg or `POLARIS_SERVER` env var before Phase 3 can work. This is a small code change but must not be forgotten.
- **GPS-absent graceful handling:** FEATURES.md anti-features note that `polaris_bridge` should handle absent GPS without crashing. This has not been verified — if the Pixhawk sends no GPS data indoors and the bridge panics on a missing field, that will surface in Phase 4 and needs a quick fix.

## Sources

### Primary (HIGH confidence)
- [ROS2 Humble Ubuntu Install — docs.ros.org](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [r2r crate — github.com/sequenceplanner/r2r](https://github.com/sequenceplanner/r2r)
- [ArduPilot Ethernet/Network Setup — ardupilot.org](https://ardupilot.org/copter/docs/common-network.html)
- [mavlink::connect — docs.rs/mavlink](https://docs.rs/mavlink/latest/mavlink/fn.connect.html)
- [Quinn TLS certificate — quinn-rs.github.io](https://quinn-rs.github.io/quinn/quinn/certificate.html)
- [rustup.rs — rustup.rs](https://rustup.rs/)

### Secondary (MEDIUM confidence)
- [Pixhawk 6X Ethernet link-down / reboot requirement — discuss.ardupilot.org](https://discuss.ardupilot.org/t/pixhawk-6x-ethernet-connection-to-mavproxy-link-down/120500)
- [Jetson Orin Ubuntu 22.04 ROS2 package conflicts — forums.developer.nvidia.com](https://forums.developer.nvidia.com/t/cannot-install-ros-2-gazebo-and-doosan-packages-on-jetson-orin-nano-ubuntu-22-04/351692)
- [Holybro Pixhawk Jetson Ethernet MAVLink setup — docs.holybro.com](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/mavlink-bridge)
- [ROS2 Humble DDS domain isolation — docs.ros.org](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)
- [JetPack 6.2.2 / Jetson Linux 36.5 — forums.developer.nvidia.com](https://forums.developer.nvidia.com/t/jetpack-6-2-2-jetson-linux-36-5-is-now-live/359622)

### Tertiary (LOW confidence)
- [r2r minimal node colcon integration — github.com/m-dahl/r2r_minimal_node](https://github.com/m-dahl/r2r_minimal_node) — IDL_PACKAGE_FILTER pattern; needs validation against actual Humble aarch64 build

---
*Research completed: 2026-02-23*
*Ready for roadmap: yes*
