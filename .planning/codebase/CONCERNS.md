# Codebase Concerns

**Analysis Date:** 2026-02-23

## Tech Debt

**No safety filter in vehicle_control_node despite SPEED_LIMIT env var:**
- Issue: `polaris_control/src/main.rs` is a pure passthrough — it reads `/input/*` and republishes to `/cmd/*` unchanged. The SPEED_LIMIT=8 env var is set in `launch/sim.sh` (line 75) but the node never reads it. There is no throttle clamping, rate limiting, or deadman switch.
- Files: `polaris_control/src/main.rs`
- Impact: Operator error or hardware glitch sends full throttle directly to the Pixhawk with no interception. The env var creates a false sense of safety — it silently does nothing.
- Fix approach: Read SPEED_LIMIT in vehicle_control_node, convert m/s to throttle fraction, clamp outgoing throttle. Or remove the env var from sim.sh to avoid confusion.

**Wire protocol is a hardcoded binary struct with no versioning:**
- Issue: The 14-byte up / 72-byte down QUIC packet format is duplicated between `polaris_wheel/src/main.rs` (`WheelPacket::to_bytes`, `VehicleDown::parse`) and `polaris_operator/src/bin/wheel_bridge_node.rs` (`parse_packet`, `VehicleDown::to_bytes`). Any field addition or reorder breaks both ends silently — bytes shift without a compile error.
- Files: `polaris_wheel/src/main.rs` lines 37-89, `polaris_operator/src/bin/wheel_bridge_node.rs` lines 23-73
- Impact: Adding a telemetry field (e.g. battery voltage) requires coordinated changes in two crates that build on different machines. A mismatch produces garbage sensor data with no error.
- Fix approach: Define the packet structs in a shared crate (e.g. `polaris_msgs` or a new `polaris_proto`) and use `bytemuck` or similar for zero-copy casting with a static assert on size.

**CmdState / Actuators struct duplicated across four crates:**
- Issue: Essentially the same struct (steering f32, throttle f32, brake f32, gear u8, handbrake bool) is independently defined in `polaris_bridge/src/main.rs` (CmdState), `polaris_control/src/main.rs` (Actuators), `polaris_sim/src/main.rs` (CmdState), and `polaris_operator/src/bin/wheel_bridge_node.rs` (WheelState).
- Files: `polaris_bridge/src/main.rs:8`, `polaris_control/src/main.rs:5`, `polaris_sim/src/main.rs:8`, `polaris_operator/src/bin/wheel_bridge_node.rs:9`
- Impact: Low risk of divergence today since the project is small, but any actuator addition (e.g. lights, horn) requires four edits.
- Fix approach: Not worth extracting until a third actuator type is added. Document the pattern so changes touch all four files.

**MAX_STEER constant defined in three places with different values:**
- Issue: The maximum steering angle is hardcoded differently across crates: `polaris_bridge/src/main.rs` line 34 uses `7.854f32` (450°), `polaris_operator/src/main.rs` line 63 uses `7.854`, `polaris_wheel/src/main.rs` line 13 uses `7.854`, but `polaris_operator/src/bin/gamepad_node.rs` line 26 uses `0.5236` (30°), and `polaris_sim/src/main.rs` line 30 uses `0.5236f32` (30°).
- Files: `polaris_bridge/src/main.rs:34`, `polaris_operator/src/main.rs:63`, `polaris_operator/src/bin/gamepad_node.rs:26`, `polaris_sim/src/main.rs:30`
- Impact: Actual range mismatch between sim bridge and real bridge. BeamNG receives steering normalized to ±30° while Pixhawk normalizes to ±450°. The same physical wheel position produces very different steering commands in sim vs real.
- Fix approach: Decide on one canonical value per vehicle type. Add a comment documenting which value is for real Polaris (450°) vs BeamNG default vehicle (~30°). Hardcode per-binary rather than having a "shared" constant that drifts.

**sync.sh uses scp, not rsync — does not delete remote files:**
- Issue: `sync.sh` copies files with `scp -r` which cannot delete files removed locally. Stale binaries, old `.rs` source files, and deleted `.msg` definitions accumulate on fastgpu.
- Files: `sync.sh`
- Impact: Removing a binary target or message type requires manual SSH cleanup. The MEMORY.md documents this as a known pitfall. Old build artifacts can cause colcon to use stale generated code.
- Fix approach: Replace `scp -r` with `rsync --delete` to keep the remote in sync with local.

**Hardcoded hostname "fastgpu" in polaris_sim:**
- Issue: `polaris_sim/src/main.rs` line 28 has `let host = "fastgpu".to_string()`. BeamNG runs on Windows on a machine assumed to be named "fastgpu". No env var override.
- Files: `polaris_sim/src/main.rs:28`
- Impact: Cannot run beamng_bridge_node against a differently-named host without recompiling. Unlike pixhawk_bridge which reads `PIXHAWK_ADDR` from env.
- Fix approach: Add `let host = std::env::var("BEAMNG_HOST").unwrap_or_else(|_| "fastgpu".into())` matching the pattern already used in polaris_bridge.

**BeamNG protocol version hardcoded with no mismatch recovery:**
- Issue: `polaris_sim/src/bridge.rs` line 6 hardcodes `PROTOCOL_VERSION: &str = "v1.26"`. On mismatch the bridge sets state to Error and returns, causing the main loop to disconnect and retry every 2 seconds forever.
- Files: `polaris_sim/src/bridge.rs:6`, `polaris_sim/src/bridge.rs:73-76`
- Impact: If BeamNG is updated, the node will silently spin in a retry loop. The log message says "Protocol mismatch" but only appears at warn level and may be missed.
- Fix approach: No code change needed, but the warning message should include clear instructions. Already acceptable for the project's experimental stage.

## Known Bugs

**pixhawk_connected flag never set false from IMU timeout correctly:**
- Symptoms: `DashboardState.pixhawk_connected` in `polaris_operator/src/main.rs` line 375 is set to `true` when any IMU message arrives. It is only cleared on the 1Hz rate sampler (line 429) when both imu_count and gps_count are 0. But `pixhawk_connected` is not actually rendered in the dashboard — `dot(r.state_hz)` is labeled "PIXHAWK" but tests `/vehicle/state` Hz, not the `pixhawk_connected` field.
- Files: `polaris_operator/src/main.rs:375`, `polaris_operator/src/main.rs:235`
- Trigger: The PIXHAWK indicator in the dashboard goes green when `/vehicle/state` publishes, not when Pixhawk IMU is actually streaming.
- Workaround: Not misleading in practice since state only publishes when bridge is running.

**VehicleState.stamp is never populated:**
- Symptoms: `VehicleState.msg` defines `builtin_interfaces/Time stamp` but neither `polaris_bridge/src/main.rs` nor `polaris_sim/src/main.rs` sets it — `r2r::polaris_msgs::msg::VehicleState::default()` leaves stamp as zero.
- Files: `polaris_bridge/src/main.rs:117-129`, `polaris_sim/src/main.rs:113-125`
- Trigger: Any consumer checking `msg.stamp` for message age or latency will always see epoch zero.
- Workaround: No consumers currently use stamp, so no operational impact.

**E-stop from gamepad does not latch — throttle can immediately be re-applied:**
- Symptoms: In `polaris_operator/src/gamepad.rs` line 82, the Start button sets throttle=0 and brake=1.0 but does not set any persistent "stopped" flag. The next axis read (line 96) overwrites throttle with the pedal value immediately.
- Files: `polaris_operator/src/gamepad.rs:80-83`, `polaris_operator/src/gamepad.rs:96`
- Trigger: Press Options (e-stop), then physically press throttle pedal — vehicle accelerates again on the very next poll cycle (20ms).
- Workaround: Physically releasing the throttle pedal before pressing Options achieves the intended effect.

## Security Considerations

**QUIC TLS certificate verification is completely disabled on the client:**
- Risk: `polaris_wheel/src/main.rs` implements `SkipServerVerification` which accepts any certificate from any server. The `insecure_client_config()` function is explicitly named but the risk is that any host on the network advertising port 9876 can receive wheel commands and send fake telemetry.
- Files: `polaris_wheel/src/main.rs:150-186`
- Current mitigation: The system operates on a private WiFi LAN. QUIC still encrypts the channel, preventing passive eavesdropping.
- Recommendations: Pin the server's self-signed cert on the client. The cert is already generated and stored at `~/.config/polaris/cert.der` on the server side — copy it to the Mac and verify against it.

**Self-signed cert is generated fresh if missing, with no key pinning:**
- Risk: `polaris_operator/src/bin/wheel_bridge_node.rs` lines 76-94 generate a new cert to `~/.config/polaris/` if none exists. If the remote directory is wiped (e.g. after OS reinstall), a new cert is silently generated. The client (which skips verification) connects anyway, so this is transparent to the user but means there is no TOFU (trust on first use) mechanism.
- Files: `polaris_operator/src/bin/wheel_bridge_node.rs:76-94`
- Current mitigation: Private LAN only.
- Recommendations: Acceptable at experimental stage. Document the cert location so it can be backed up.

**Force arm bypasses all pre-arm safety checks:**
- Risk: `polaris_bridge/src/pixhawk.rs` line 88 passes `param2: 21196.0` to ARM_DISARM, which is the ArduPilot magic number to force-arm regardless of safety checks (GPS lock, compass, etc). This arms the vehicle even if the Pixhawk is in an unknown or unsafe state.
- Files: `polaris_bridge/src/pixhawk.rs:88`
- Current mitigation: Only used in SITL during development. Acceptable for SITL; dangerous for real vehicle deployment.
- Recommendations: Before using with real hardware, add a flag or env var to disable force-arm and let ArduPilot's pre-arm checks run.

## Performance Bottlenecks

**Blocking MAVLink recv holds Mutex across an entire message receive:**
- Problem: `polaris_bridge/src/main.rs` line 104 spawns a std::thread for MAVLink receive. Inside the loop (lines 106-111), it acquires the Mutex on `pixhawk` to check if connected, then drops the guard before calling `iface.recv_telemetry()`. This is correct. However, `iface.send_rc_override()` (lines 115, 144) is called while holding the `cmd` Mutex inside the telemetry receive loop. The send is synchronous and unbuffered — a slow MAVLink write blocks the cmd Mutex, which blocks the async subscriber tasks.
- Files: `polaris_bridge/src/main.rs:104-163`
- Cause: MAVLink's `conn.recv()` is blocking (no timeout configured). If the Pixhawk stops sending, the thread blocks indefinitely, and RC overrides stop being sent.
- Improvement path: Call `conn.set_recv_timeout(Some(Duration::from_millis(100)))` after connecting, or use the mavlink async API.

**polaris_sim sends control and polls state sequentially in one loop iteration:**
- Problem: `polaris_sim/src/main.rs` lines 98-132 run `send_control().await` then `poll_state().await` back-to-back on each tick. Each is a full TCP round-trip (send + recv). At 50Hz (20ms budget) two round-trips to a remote host adds latency that could miss the tick window under load.
- Files: `polaris_sim/src/main.rs:98-132`
- Cause: Simple sequential design; BeamNG protocol requires request/response.
- Improvement path: Acceptable for simulation. If latency becomes an issue, pipeline the requests or reduce poll frequency.

## Fragile Areas

**polaris_control subscribers built from a Vec iterator — order coupling:**
- Files: `polaris_control/src/main.rs:34-68`
- Why fragile: Lines 34-36 create a Vec of 3 subscriptions `[steering, throttle, brake]` then consume them with `steer_sub.next().unwrap()` three times. The handler that runs for each depends entirely on iteration order matching the vec literal order. A reorder of the vec elements swaps steering for throttle silently.
- Safe modification: Named variables are already used for gear and handbrake. The Vec pattern was used only for the f32 trio. If a fourth f32 actuator is added, extend the pattern carefully or switch to named bindings like the other two.
- Test coverage: No tests exist for this mapping.

**BeamNG bridge shares `next_id` counter between GE and vehicle TCP streams:**
- Files: `polaris_sim/src/bridge.rs:186-188`
- Why fragile: `send_ge` and `send_veh` both mutate `&mut self.next_id`, so IDs are globally incrementing across both connections. The BeamNG protocol appears to match responses by connection (not ID), so this probably works, but if BeamNG ever validates that ID sequences restart per connection, messages will be rejected.
- Safe modification: Keep as-is unless BeamNG protocol documentation clarifies ID scoping.
- Test coverage: No protocol-level tests.

**IMU units from RAW_IMU are in milli-g and milli-rad/s, not SI:**
- Files: `polaris_bridge/src/pixhawk.rs:138-139`
- Why fragile: `RAW_IMU` fields (xacc, yacc, zacc) are in milli-g (9.81 mm/s²), and gyro values are in milli-rad/s. Dividing by 1000.0 gives g and rad/s, not m/s². The published `sensor_msgs/Imu` message convention expects m/s² for linear_acceleration.
- Impact: IMU acceleration display in the dashboard is in g-units, not m/s². For the current use case (display only) this is cosmetic. Any future consumer that uses the IMU for control or localization will get incorrect values.
- Safe modification: Multiply acc by 9.81 after dividing by 1000: `data.xacc as f32 / 1000.0 * 9.81`.
- Test coverage: None.

## Scaling Limits

**No watchdog or failsafe for loss of QUIC connection:**
- Current capacity: Single QUIC bidirectional stream per connection. If polaris_wheel disconnects, wheel_bridge_node publishes the last received state at 50Hz indefinitely.
- Limit: If the Mac disconnects mid-drive (WiFi dropout), the vehicle continues executing the last command (e.g. full throttle) until a node is manually killed.
- Scaling path: Add a timestamp to the QUIC packet. In wheel_bridge_node, if no packet received within 200ms, publish throttle=0 brake=1 to `/input/*`.

**One vehicle only — BeamNG bridge takes first vehicle found:**
- Current capacity: `polaris_sim/src/bridge.rs` lines 91-95 call `GetCurrentVehicles` and take `m.first()` — always the first vehicle in the map.
- Limit: Multi-vehicle scenarios impossible without code change.
- Scaling path: Accept a vehicle ID env var.

## Dependencies at Risk

**r2r 0.9 depends on ROS2 installation at build time:**
- Risk: `r2r` generates Rust bindings from installed ROS2 message definitions via build.rs. If the ROS2 version on fastgpu changes (Jazzy → next LTS), the bindings regenerate and any API changes cause compile failures that may not be caught until the next sync+build cycle.
- Impact: Breaks all four ROS2 crates (polaris_bridge, polaris_control, polaris_operator, polaris_sim) simultaneously.
- Migration plan: Pin the ROS2 installation version on fastgpu. Document the Jazzy dependency explicitly.

**mavlink 0.14 uses REQUEST_DATA_STREAM which is deprecated in MAVLink 2:**
- Risk: `polaris_bridge/src/pixhawk.rs` line 96 uses `REQUEST_DATA_STREAM` (MAVLink message #66), which ArduPilot still supports but MAVLink 2 marks as deprecated in favor of `SET_MESSAGE_INTERVAL` per-message.
- Impact: Works with current ArduPilot versions. Future ArduPilot releases may drop support.
- Migration plan: Not urgent. Switch to SET_MESSAGE_INTERVAL if ArduPilot deprecates REQUEST_DATA_STREAM.

## Missing Critical Features

**No deadman switch / connection-loss failsafe:**
- Problem: If the operator's QUIC connection drops, the last commanded state (possibly full throttle) continues forwarding to the Pixhawk indefinitely. No timeout or neutral command is issued.
- Blocks: Safe real-vehicle deployment. Currently only acceptable in SITL.

**No heartbeat sent to Pixhawk:**
- Problem: ArduPilot expects to receive a regular GCS heartbeat (HEARTBEAT message at ~1Hz) to confirm the controlling station is alive. `polaris_bridge/src/pixhawk.rs` never sends one. In MANUAL mode ArduPilot may not enforce this, but in other modes the vehicle may disarm or refuse commands without it.
- Files: `polaris_bridge/src/pixhawk.rs`
- Blocks: Robust autopilot integration. Required before autonomous mode work.

## Test Coverage Gaps

**Zero tests across all crates:**
- What's not tested: PWM mapping math in `polaris_bridge/src/pixhawk.rs` (`value_to_pwm`, `value_to_pwm_centered`), packet serialization/deserialization in `polaris_wheel/src/main.rs` and `polaris_operator/src/bin/wheel_bridge_node.rs`, pedal normalization in `polaris_operator/src/gamepad.rs` and `polaris_wheel/src/main.rs`, BeamNG frame protocol parsing in `polaris_sim/src/bridge.rs`.
- Files: All `src/` files — no `#[test]` or `#[cfg(test)]` blocks exist.
- Risk: PWM math errors or packet byte-offset bugs would only surface during live hardware or simulation runs. The `value_to_pwm_centered` asymmetric formula (different coefficients for positive and negative) is particularly worth unit-testing.
- Priority: High for `value_to_pwm` and packet parse/serialize. Low for UI/TUI code.

---

*Concerns audit: 2026-02-23*
