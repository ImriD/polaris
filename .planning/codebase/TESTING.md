# Testing Patterns

**Analysis Date:** 2026-02-23

## Test Framework

**Runner:** Rust's built-in `cargo test`

**Assertion Library:** Standard `assert!` / `assert_eq!` macros (Rust stdlib)

**Run Commands:**
```bash
cargo test                          # Run all unit tests
cargo test -- --ignored             # Run integration tests marked #[ignore]
RUST_LOG=debug cargo test           # Run tests with logging
```

## Test File Organization

**Location:** No dedicated test files exist yet. The only test-adjacent file is `polaris_wheel/src/test_detect.rs`, which is a standalone diagnostic binary (`fn main()`), not a test module.

**Pattern in codebase:** Zero `#[cfg(test)]` blocks, zero `#[test]` functions, zero `mod tests` declarations found across all five crates.

**Structure:**
```
polaris/
├── polaris_bridge/src/         # No tests
├── polaris_control/src/        # No tests
├── polaris_operator/src/       # No tests
├── polaris_sim/src/            # No tests
└── polaris_wheel/src/
    └── test_detect.rs          # Diagnostic binary, not a test
```

## Diagnostic Binary Pattern

The one file that performs runtime verification is a standalone `[[bin]]` entry run manually:

`polaris_wheel/src/test_detect.rs`:
```rust
fn main() {
  let gilrs = gilrs::Gilrs::new().unwrap();
  let count = gilrs.gamepads().count();
  println!("Gamepads found: {}", count);
  for (_id, gp) in gilrs.gamepads() {
    println!("  {} (mapping: {:?})", gp.name(), gp.mapping_source());
  }
}
```

This is the only testing pattern in the codebase — run-it-and-observe rather than automated assertions.

## Mocking

**Framework:** None. No mock infrastructure exists.

**Current approach:** Live hardware or live network connections are required to exercise any code path. There is no dependency injection or trait abstraction that would enable substituting test doubles.

## Fixtures and Factories

**Test Data:** None. No fixture files, no factory functions.

## Coverage

**Requirements:** None enforced. No coverage tooling configured.

## Test Types

**Unit Tests:** Not present.

**Integration Tests:** Not present as formal tests. Integration is verified by running binaries against live systems (BeamNG simulator on `fastgpu`, Pixhawk hardware via serial, G923 wheel via SDL2/gilrs).

**E2E Tests:** Not present. System-level verification is done by running the full node graph and observing ROS topic output with `ros2 topic echo`.

## What Should Be Tested (Per Project Conventions)

Per the project's `CLAUDE.md`:
- **Parsing functions** need unit tests — `parse_packet`, `VehicleDown::parse`, `WheelPacket::to_bytes`, `value_to_pwm`, `value_to_pwm_centered`, `pedal_to_normalized`, `steering_to_normalized`, `map_get`, `parse_vec3`, `parse_vec4`, `normalize_pedal`
- **API integrations** (MAVLink, BeamNG bridge, QUIC) need integration tests marked `#[ignore]` for live calls

## How to Add Tests (When Adding)

**Unit tests** go in the same file as the code under test, in a `mod tests` block:
```rust
#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_value_to_pwm() {
    assert_eq!(value_to_pwm(0.0, 1000, 2000), 1000);
    assert_eq!(value_to_pwm(1.0, 1000, 2000), 2000);
    assert_eq!(value_to_pwm(0.5, 1000, 2000), 1500);
  }

  #[test]
  fn test_parse_packet_roundtrip() {
    // serialize then deserialize
  }
}
```

**Integration tests** that require live hardware or network, marked `#[ignore]` so they don't run in CI:
```rust
#[test]
#[ignore] // requires live Pixhawk on serial port
fn test_pixhawk_connect() {
  let iface = PixhawkInterface::connect("serial:/dev/ttyUSB0:921600").unwrap();
  // ...
}
```

**Priority areas** for adding test coverage:
1. `polaris_bridge/src/pixhawk.rs` — `value_to_pwm`, `value_to_pwm_centered` (pure math, trivial to test)
2. `polaris_operator/src/bin/wheel_bridge_node.rs` — `parse_packet` / `VehicleDown::to_bytes` roundtrip
3. `polaris_wheel/src/main.rs` — `pedal_to_normalized`, `steering_to_normalized` (normalization logic with edge cases)
4. `polaris_sim/src/bridge.rs` — `map_get`, `parse_vec3`, `parse_vec4` (msgpack parsing helpers)
5. `polaris_operator/src/gamepad.rs` — `normalize_pedal`, `apply_deadzone`

---

*Testing analysis: 2026-02-23*
