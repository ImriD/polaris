# Coding Conventions

**Analysis Date:** 2026-02-23

## Naming Patterns

**Files:**
- Snake_case module files: `main.rs`, `bridge.rs`, `pixhawk.rs`, `gamepad.rs`
- Test binaries live in `src/bin/` as descriptive snake_case names: `wheel_bridge_node.rs`, `gamepad_node.rs`
- Utility/diagnostic binaries: `test_detect.rs` in `src/` as a standalone `fn main()`

**Structs:**
- PascalCase for all types: `CmdState`, `WheelState`, `DashboardState`, `BeamNGBridge`, `PixhawkInterface`, `VehicleDown`, `TopicRates`, `RateSnapshot`
- State structs named `*State` (per-crate shared mutable state): `CmdState`, `WheelState`, `DashboardState`, `Actuators`
- Connection/protocol structs named for their domain: `BeamNGBridge`, `PixhawkInterface`, `VehiclePhysics`

**Enums:**
- PascalCase variants: `ConnectionState::Disconnected`, `ConnectionState::Connected`, `ConnectionState::Ready`, `ConnectionState::Error`
- `TelemetryUpdate::Position { .. }`, `TelemetryUpdate::Imu { .. }` — variants with named fields

**Functions:**
- Snake_case: `async_main`, `ros_spin`, `parse_packet`, `load_or_generate_cert`, `handle_connection`, `value_to_pwm`, `pedal_to_normalized`, `steering_to_normalized`, `map_get`, `parse_vec3`, `parse_vec4`

**Constants:**
- SCREAMING_SNAKE_CASE: `GEAR_PWM`, `HANDBRAKE_ENGAGED`, `HANDBRAKE_RELEASED`, `STEER_INCREMENT`, `MAX_STEER`, `PACKET_HZ`, `DEADZONE`, `AXIS_STEERING`, `BTN_RIGHT_PADDLE`, `LISTEN_PORT`, `PROTOCOL_VERSION`

**Variables:**
- Short single-letter locals for cloned `Arc` handles: `c`, `st`, `r`, `px`, `vd`, `ws`
- Descriptive names for primary values: `wheel_state`, `vehicle_down`, `tcp_rx_count`, `pub_count`

## Code Style

**Indentation:** 2 spaces (consistent across all files)

**Line length:** Lines frequently reach 100-150 characters; subscriber closures are intentionally dense and kept on one line when they fit, e.g.:
```rust
steer_sub.for_each(|msg| { c.lock().unwrap().steering = msg.data; std::future::ready(()) }).await;
```

**Formatting tool:** Not detected (no `rustfmt.toml` or `.rustfmt` config). Code is hand-formatted and consistent.

**Struct initialization:** All fields on one line when they fit the 150-char limit:
```rust
Self { steering: 0.0, throttle: 0.0, brake: 0.0, gear: 3, handbrake: false }
```
Multi-line when struct is large (`DashboardState::new`, `BeamNGBridge::new`).

**`..Default::default()`:** Used for large MAVLink message structs to fill unused fields:
```rust
mavlink::ardupilotmega::RC_CHANNELS_OVERRIDE_DATA { chan1_raw: 1500, ..Default::default() }
```

## Import Organization

**Order:** Standard library → external crates — no blank-line separation between groups.

**Pattern:** `use` at the top of each file, fully qualified where needed. No glob imports except `ratatui::prelude::*` and `ratatui::widgets::*` in TUI files.

**Module declarations:** `mod bridge;` / `mod pixhawk;` / `mod gamepad;` at the top of `main.rs`, above all `use` statements.

**Cross-crate module reference:** `#[path = "../gamepad.rs"] mod gamepad;` in bin files to reuse sibling module code.

**Path aliases:** None. All paths are explicit.

## Error Handling

**Strategy:** `?` propagation throughout. Function signatures return `Result<_, Box<dyn std::error::Error>>` or the `Send + Sync` variant for async contexts.

**Entry point signature:** Every `main` or `async_main` returns `Result<(), Box<dyn std::error::Error>>`.

**Fire-and-forget publishes:** ROS publish calls always use `let _ =` to explicitly discard errors — publishing is best-effort:
```rust
let _ = state_pub.publish(&vs);
```

**Connection errors:** Logged with `log::warn!` and trigger a retry loop — never propagated up:
```rust
Err(e) => log::warn!("Connect failed: {e}"),
```

**Protocol mismatches:** Returned as heap-allocated string errors via `.into()`:
```rust
return Err(format!("Protocol mismatch: expected {}, got {:?}", PROTOCOL_VERSION, version).into());
```

**Missing data:** Use `Option` chaining with `.and_then()` and `.ok_or("message")?`:
```rust
let vid = vehicles.and_then(|m| m.first()).and_then(|(k, _)| k.as_str()).ok_or("No vehicles found")?;
```

**Unreachable panics:** `unwrap()` is used freely on `Mutex::lock()` (cannot fail unless poisoned) and on known-safe index operations. No defensive checks for impossible conditions.

## Logging

**Framework:** `log` crate with `env_logger` backend. Initialized with `env_logger::init()` at the start of every `main`.

**Levels used:**
- `log::info!` — state transitions, connections established, mode changes, startup messages
- `log::warn!` — connection failures, disconnects, errors that trigger retries
- `log::error!` — fatal background thread errors

**Format:** Plain string interpolation with `{}` or `{e}` shorthand for errors:
```rust
log::info!("Connecting to Pixhawk: {}", addr);
log::warn!("Connect failed: {e}");
log::error!("ROS2 thread error: {}", e);
```

## Comments

**When to comment:** Section headers for logical blocks using `// Comment —` style:
```rust
// Subscribe to /cmd/* (filtered commands from vehicle_control_node)
// Connection retry at 2Hz
// Handle /cmd/* — store latest values
// Wire protocol ---
```

**Doc comments:** Not used (`///` absent from codebase). No rustdoc.

**Inline clarifications:** Used for magic numbers and protocol details:
```rust
let max_steering_angle = 7.854f32; // 450 degrees in radians
param1: 1.0, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
chan1_raw: 1500, // steering center
```

**No TODO comments** — per project convention, incomplete ideas are not recorded as comments.

## Async Pattern

**Runtime setup:** Every node entry point uses the same pattern:
```rust
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  let local = tokio::task::LocalSet::new();
  local.run_until(async_main()).await
}
```
`current_thread` + `LocalSet` allows `spawn_local` for r2r subscribers (which are `!Send`).

**Subscriber tasks:** Each ROS topic subscriber gets its own `tokio::task::spawn_local` with a `for_each` closure that updates shared state through a `Mutex`:
```rust
let c = cmd.clone();
tokio::task::spawn_local(async move {
  steer_sub.for_each(|msg| { c.lock().unwrap().steering = msg.data; std::future::ready(()) }).await;
});
```

**Spin loop:** All nodes end with a spin loop that yields to the executor each iteration:
```rust
loop {
  node.spin_once(std::time::Duration::from_millis(10));
  tokio::time::sleep(std::time::Duration::from_millis(1)).await;
}
```

**Blocking MAVLink:** Blocking I/O runs in `std::thread::spawn` to avoid blocking the async runtime.

## Shared State Pattern

**Type:** `Arc<Mutex<T>>` for all shared mutable state across tasks and threads.

**Clone naming:** Short single-letter aliases for clones passed into closures (`c`, `st`, `px`, `vd`, `r`).

**Lock discipline:** Locks are held for the minimum scope. `drop(guard)` is called explicitly when the lock must be released before an I/O await:
```rust
let cmd = c.lock().unwrap();
// use cmd fields
drop(cmd);
// then do I/O
```

**Atomic counters:** `Arc<AtomicU64>` for high-frequency counters (message rates, packet counts) to avoid mutex contention:
```rust
r.input_steering.fetch_add(1, Ordering::Relaxed);
```

## Module Design

**Exports:** `pub` on types and methods that cross module boundaries. Internal helpers are private (e.g., `value_to_pwm`, `map_get`, `send_ge`, `send_veh`).

**Module per subsystem:** Each external interface gets its own module file: `pixhawk.rs` in `polaris_bridge`, `bridge.rs` in `polaris_sim`, `gamepad.rs` in `polaris_operator`.

**No barrel files:** Each crate has a single `main.rs` entry point; modules are declared with `mod` in `main.rs`.

---

*Convention analysis: 2026-02-23*
