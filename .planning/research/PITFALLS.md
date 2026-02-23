# Pitfalls — Jetson ROS2 Table Rig

**Research Date:** 2026-02-23
**Scope:** Common mistakes deploying ROS2 Humble + Rust (r2r) on Jetson aarch64, connecting to Pixhawk over Ethernet MAVLink, and running a QUIC server.

---

## Summary

Nine categories of pitfall, ordered by how early they will bite you. The first three (environment, build, MAVLink) will block the whole project if ignored. The remaining six are operational traps that compound once the stack is "running."

---

## P1 — ROS2 environment variables die with nohup

**What goes wrong:**
`nohup` on the Jetson spawns a new login shell that has not sourced `/opt/ros/humble/setup.bash` or `~/ros2_ws/install/setup.bash`. Nodes launch but immediately crash with `librcl_action.so: cannot open shared object file` or silently fail to discover topics. This is already a known issue from fastgpu, but the Humble install path differs from Jazzy.

**Warning signs:**
- Node PID appears in `ps` but produces no `/cmd/*` or `/vehicle/*` topics
- Log shows `cannot open shared object file: No such file or directory` for any `librcl_*.so`
- `ros2 topic list` returns empty even when nodes are running

**Prevention:**
Pass the full environment inline rather than relying on shell sourcing:
```bash
ssh jetson "
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nohup ~/polaris/target/release/pixhawk_bridge_node > /tmp/pixhawk.log 2>&1 &
"
```
Or use a wrapper script on the Jetson that sources both files before exec-ing the binary — this is the cleanest long-term pattern.

**Phase:** Provisioning / launch script authoring.

---

## P2 — Stale r2r message bindings after polaris_msgs changes

**What goes wrong:**
r2r generates Rust bindings at `cargo build` time by calling into the C introspection libraries from the installed message packages. If `polaris_msgs` is changed and colcon is rerun, but `cargo clean -p r2r_msg_gen` is not run, the old compiled bindings remain and the build silently uses the stale types. This produces subtle runtime panics or silent field mismatches — not a compile error.

**Warning signs:**
- `colcon build --packages-select polaris_msgs` succeeded but `VehicleState` field count changed
- Cargo build succeeds but runtime behavior is wrong for telemetry fields
- On fastgpu, the known fix was `rm -rf build/polaris_msgs install/polaris_msgs && colcon build`

**Prevention:**
Any time `.msg` files change: run `cargo clean -p r2r_msg_gen` before `cargo build`. Document this in the rig launch script as a comment. Also: set `IDL_PACKAGE_FILTER` in `.cargo/config.toml` to restrict binding generation to only `polaris_msgs` and `std_msgs` — this both speeds up builds and reduces the surface area of stale bindings from unrelated system messages.

**Phase:** First build on Jetson; any subsequent `.msg` file edit.

---

## P3 — Pixhawk Ethernet MAVLink UDP client/server mismatch

**What goes wrong:**
ArduPilot's Ethernet MAVLink (NET_ENABLE=1) requires both ends to agree on who is client and who is server. If Pixhawk is configured as UDP Server (listens on 14550) and the bridge uses `udpin:0.0.0.0:14550` (also listening), neither side connects. Conversely if both are configured as clients they each send into the void. A reboot is required after changing `NET_P1_TYPE` for the new setting to take effect — many people change the parameter and test without rebooting, then conclude the feature is broken.

**Warning signs:**
- Bridge process starts, no MAVLink heartbeat received within 5 seconds
- `ping 192.168.88.242` works but MAVLink messages never arrive
- Pixhawk web interface or QGroundControl shows `NET_ENABLE=1` but link is down

**Prevention:**
Establish the convention at setup time: Pixhawk configured as UDP Client (`NET_P1_TYPE=1`, destination `192.168.88.175:14550`), bridge uses `udpin:0.0.0.0:14550` (listens). This is the companion-computer idiom — Pixhawk reaches out, bridge receives. Reboot Pixhawk after every `NET_P*` parameter change. Confirm with `tcpdump -i any udp port 14550` on the Jetson before starting the bridge process.

**Phase:** Initial Pixhawk network setup, before any Rust code runs.

---

## P4 — r2r colcon build compiles all system message packages

**What goes wrong:**
Without `IDL_PACKAGE_FILTER`, r2r compiles Rust bindings for every `.msg` and `.srv` file in the entire sourced workspace — including all of `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`, and hundreds of others. On an aarch64 device this can take 20–40 minutes per build. This makes iteration miserable and masks real errors in output noise.

**Warning signs:**
- `cargo build` takes more than 10 minutes
- Build output shows `Compiling r2r_msg_gen` followed by hundreds of message type compilations
- System heat or fan noise during a dependency step before any application code is reached

**Prevention:**
In `.cargo/config.toml` at the workspace root:
```toml
[env]
IDL_PACKAGE_FILTER = "polaris_msgs:std_msgs:sensor_msgs:mavros_msgs"
```
List only what the crates actually use. Note: r2r does no automatic dependency resolution for nested message types, so include all transitive packages explicitly. Verify the filter works by timing a cold build — it should finish in under 5 minutes.

**Phase:** First build on Jetson.

---

## P5 — ROS2 DDS multicast flooding the Teltonika network

**What goes wrong:**
ROS2 Humble uses FastDDS by default, which sends multicast discovery packets to find peers. All devices on the 192.168.88.x subnet receive these packets — including the Mac, the Pixhawk, and any other devices on the Teltonika router. This does not cause connectivity failures but adds unnecessary chatter, can confuse poorly-behaved Pixhawk UDP handling, and will cause problems if multiple ROS2 systems share the network later.

**Warning signs:**
- Wireshark on the Mac shows RTPS multicast packets from 192.168.88.175
- `ros2 topic list` on the Mac (if ROS2 is installed) shows Jetson topics without any configuration
- Pixhawk logs show unexpected UDP traffic on non-MAVLink ports

**Prevention:**
Set `ROS_LOCALHOST_ONLY=1` in the launch environment since all ROS2 nodes run on the Jetson itself — DDS traffic never needs to leave the machine. Alternatively, set `ROS_DOMAIN_ID` to a non-zero value (e.g., `42`) to namespace the domain and reduce accidental interference with any other ROS2 system on the network.

**Phase:** First launch on Jetson.

---

## P6 — r2r links against wrong ROS2 installation

**What goes wrong:**
If both a system ROS2 install and a colcon overlay workspace are present, r2r uses pkg-config to find the correct libraries. If `setup.bash` is not sourced before `cargo build`, r2r may link against the system-level install and miss the custom `polaris_msgs` types, producing `failed to find message type VehicleState` at runtime. On the Jetson, if a partial ROS2 install exists or a previous colcon build is stale, pkg-config resolves to unexpected paths.

**Warning signs:**
- `cargo build` succeeds but running the binary prints `panicked at: failed to find type for VehicleState`
- `pkg-config --cflags rcl` points to a path that does not contain `polaris_msgs`
- Build log shows r2r finding messages but skipping `VehicleState` in the generated enum

**Prevention:**
Always build with the full environment:
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cargo build --release
```
Verify `echo $AMENT_PREFIX_PATH` includes both `/opt/ros/humble` and `~/ros2_ws/install/polaris_msgs` before building. Never run bare `cargo build` without the sourced environment — make this impossible by wrapping it in a script that sources first.

**Phase:** First build on Jetson; any rebuild after colcon workspace changes.

---

## P7 — QUIC server TLS certificate not trusted by Mac client

**What goes wrong:**
quinn requires TLS 1.3 for all connections. If the server uses a self-signed certificate generated at startup (the common pattern with `rcgen`), the Mac client must explicitly load and trust that certificate. If the certificate is regenerated on each server restart (e.g., generated in memory), the client's pinned cert is immediately invalid and the connection is refused with a TLS error. Additionally, the certificate must match the server's IP address or a configured domain name; a mismatch causes handshake failure even if the cert is otherwise trusted.

**Warning signs:**
- `polaris_wheel` logs a TLS or certificate error on connection attempt
- `quinn` client error: `Custom { kind: InvalidData, error: "peer certificate has no IP SANs" }`
- Connection succeeds on first launch but fails after Jetson restart

**Prevention:**
Generate the certificate once and persist it to disk (e.g., `/etc/polaris/server.{cert,key}`). Include the Jetson's LAN IP (`192.168.88.175`) as a Subject Alternative Name (SAN) in the certificate. Bundle the cert (public half only) with `polaris_wheel` or load it from an env var pointing to a local file. This avoids regeneration-on-restart and IP mismatch problems simultaneously.

**Phase:** QUIC integration, before first Mac-to-Jetson connection attempt.

---

## P8 — JetPack OpenCV package conflicts block ROS2 apt install

**What goes wrong:**
NVIDIA ships a JetPack-specific OpenCV build (`libopencv4tegra`) that conflicts with the Ubuntu `libopencv-dev` package that many ROS2 packages list as a dependency. When running `apt install ros-humble-desktop` or installing `ros-humble-*` packages, apt may fail with dependency conflicts or install both, leaving broken shared library references. The desktop metapackage is the most common trigger — `ros-humble-ros-base` avoids most of this.

**Warning signs:**
- `apt install ros-humble-desktop` fails with `conflicting packages` or `held broken packages`
- `dpkg` warnings about files list missing for `libopencv*`
- `colcon build` later fails with CMake errors referencing OpenCV library files that do not exist

**Prevention:**
Install `ros-humble-ros-base` (not `ros-humble-desktop`) on the Jetson — this avoids visualization tools and their heavy OpenCV dependencies. Only add specific packages actually needed: `ros-humble-std-msgs`, `ros-humble-sensor-msgs`, `ros-humble-nav-msgs`. Avoid `rosdep install --from-paths` as it pulls in optional visualization deps. Run `apt list --installed | grep opencv` before starting to check what JetPack left behind.

**Phase:** Jetson provisioning, before ROS2 installation.

---

## P9 — Pixhawk MAVLink stream rates not configured for Ethernet

**What goes wrong:**
The Pixhawk's Ethernet MAVLink port does not automatically stream telemetry at the same rate as serial ports. The default `MAV_2_MODE` for the Ethernet port is often set to a restricted profile that streams no messages or only heartbeat. The bridge expects `GLOBAL_POSITION_INT` and `RAW_IMU` at 10Hz — without explicit stream rate configuration, the `/vehicle/state`, `/vehicle/gps`, and `/vehicle/imu` topics will be empty and the TUI dashboard will show zero-Hz telemetry.

**Warning signs:**
- Bridge connects, heartbeat received, but no `/vehicle/*` topics publish
- `ros2 topic hz /vehicle/state` shows `no new messages`
- QGroundControl shows the vehicle connected but no position data in the Polaris dashboard

**Prevention:**
After the Ethernet MAVLink link is established, configure the second port's stream rates explicitly. In QGroundControl's MAVLink Console or via `mavproxy`:
```
param set MAV_2_MODE 2        # onboard profile (enables full telemetry)
param set MAV_2_RATE 10       # 10Hz stream rate
```
Or use `mavlink set-stream-rates` from the companion computer once the bridge connects. Add a verification step in the launch script: wait for a heartbeat, then request stream rates using the `REQUEST_DATA_STREAM` MAVLink message. Document the parameter values in the provisioning checklist so they survive a Pixhawk parameter reset.

**Phase:** First hardware integration test, after bridge connects successfully.

---

## Quick Reference

| # | Pitfall | Detected By | Phase |
|---|---------|-------------|-------|
| P1 | nohup drops ROS2 env vars | `librcl_*.so` not found | Launch script |
| P2 | Stale r2r message bindings | Subtle runtime field mismatch | Any .msg change |
| P3 | MAVLink UDP client/server mismatch | No heartbeat received | Pixhawk setup |
| P4 | r2r builds all system messages | 20+ min build time | First Jetson build |
| P5 | DDS multicast floods LAN | Wireshark shows RTPS on Mac | First launch |
| P6 | r2r links wrong ROS2 install | VehicleState not found at runtime | First build |
| P7 | QUIC TLS cert not trusted / IP mismatch | TLS handshake failure | QUIC integration |
| P8 | JetPack OpenCV conflicts ROS2 apt | apt dependency errors | Provisioning |
| P9 | Pixhawk stream rates not configured | Empty /vehicle/* topics | Hardware integration |

---

Sources:
- [r2r minimal node colcon integration](https://github.com/m-dahl/r2r_minimal_node)
- [r2r crate — IDL_PACKAGE_FILTER and cargo clean -p r2r_msg_gen](https://github.com/sequenceplanner/r2r)
- [ArduPilot Ethernet / Network Setup](https://ardupilot.org/copter/docs/common-network.html)
- [Pixhawk 6X ethernet connection LINK Down — reboot requirement](https://discuss.ardupilot.org/t/pixhawk-6x-ethernet-connection-to-mavproxy-link-down/120500)
- [Quinn TLS certificate configuration](https://quinn-rs.github.io/quinn/quinn/certificate.html)
- [ROS2 Humble DDS domain isolation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)
- [Jetson Orin Ubuntu 22.04 ROS2 package conflicts](https://forums.developer.nvidia.com/t/cannot-install-ros-2-gazebo-and-doosan-packages-on-jetson-orin-nano-ubuntu-22-04/351692)
- [CycloneDDS multiple interfaces on Jetson](https://iroboteducation.github.io/create3_docs/setup/xml-config/)
- [Holybro Pixhawk Jetson Ethernet MAVLink setup](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/mavlink-bridge)
