# Phase 1: Jetson Provisioning - Research

**Researched:** 2026-02-23
**Domain:** ROS2 Humble + Rust (r2r) native setup on aarch64 Ubuntu 22.04
**Confidence:** HIGH

---

## Summary

This phase provisions a fresh NVIDIA Jetson (Ubuntu 22.04 Jammy, aarch64) so that all polaris Rust crates can build and run natively. The machine has no ROS2, no Rust, and 202GB free disk — a clean slate with ample resources. Native compilation (not cross-compile) is the chosen strategy: the Jetson has 16GB RAM and the r2r crate's `bindgen`-based code generation requires libclang headers from the target machine itself.

The provisioning sequence is strictly ordered. ROS2 Humble must be installed and sourced before `cargo build` is run, because r2r's build script discovers message types from the sourced ROS2 environment at compile time. Custom messages (`polaris_msgs`) must be built with `colcon` and their workspace sourced on top of `/opt/ros/humble/setup.bash` before building polaris crates, since r2r accesses the already-generated C introspection code from those packages.

The only significant platform pitfall is the JetPack OpenCV conflict: `ros-humble-desktop` depends on `libopencv-dev` from Ubuntu's apt, which conflicts with JetPack's pre-installed OpenCV. The decision to use `ros-humble-ros-base` avoids this entirely. Everything else follows standard Ubuntu 22.04 ROS2 install docs.

**Primary recommendation:** Follow the exact sequence — locale → ROS2 apt source → ros-humble-ros-base + ros-dev-tools → rustup → build dependencies → colcon build polaris_msgs → cargo build with both setups sourced.

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|-----------------|
| PROV-01 | Jetson has ROS2 Humble installed (ros-humble-ros-base from apt) | Standard Ubuntu 22.04 aarch64 Humble deb packages are officially supported; ros-humble-ros-base avoids JetPack OpenCV conflict |
| PROV-02 | Jetson has Rust toolchain installed via rustup (aarch64-native) | rustup auto-detects aarch64-unknown-linux-gnu; standard `curl \| sh` installer works on Jetson Ubuntu 22.04 |
| PROV-03 | Jetson has build dependencies installed (libclang-dev, cmake, pkg-config) | r2r explicitly requires libclang-dev for bindgen; mavlink direct-serial needs libudev-dev; gilrs needs libudev-dev |
| PROV-04 | polaris_msgs built in colcon workspace on Jetson (~/ros2_ws/) | polaris_msgs is a standard ament_cmake rosidl package; `colcon build --packages-select polaris_msgs` then source install/setup.bash |
| PROV-05 | All polaris crates compile on Jetson (polaris_bridge, polaris_operator, polaris_control) | Requires PROV-01 through PROV-04 complete, ROS2+workspace sourced, IDL_PACKAGE_FILTER already in .cargo/config.toml |
</phase_requirements>

---

## Standard Stack

### Core
| Tool | Version | Purpose | Why Standard |
|------|---------|---------|--------------|
| ros-humble-ros-base | Humble (apt) | ROS2 middleware, CLI tools, message packages | Official Ubuntu 22.04 aarch64 package; avoids JetPack OpenCV conflict vs desktop |
| ros-dev-tools | Humble (apt) | colcon, rosdep, python3-colcon-common-extensions | Meta-package that pulls colcon and all build helpers |
| rustup | latest stable | Rust toolchain installer | Official installer; auto-detects aarch64; manages toolchain version |
| r2r | 0.9 (Cargo.toml) | Rust ↔ ROS2 bindings | Already in use; no ROS2 build infra needed, just `cargo build` |
| colcon | via ros-dev-tools | Build polaris_msgs ament_cmake package | Standard ROS2 build tool for custom message generation |

### Build Dependencies (apt)
| Package | Purpose | Required By |
|---------|---------|-------------|
| libclang-dev | C header parsing for bindgen | r2r (mandatory, documented) |
| cmake | CMake build system | polaris_msgs colcon build |
| pkg-config | Library discovery | r2r, serialport (mavlink direct-serial) |
| libudev-dev | udev library headers | mavlink direct-serial feature, gilrs gamepad |
| build-essential | gcc, make, etc. | General Rust native compilation |
| python3-pip | Python package manager | colcon internal needs |

### Already Configured (no changes needed)
| Item | Location | Notes |
|------|----------|-------|
| IDL_PACKAGE_FILTER | `.cargo/config.toml` | Already set to `polaris_msgs;std_msgs;std_srvs;sensor_msgs;geometry_msgs;builtin_interfaces` |
| polaris_msgs package | `polaris_msgs/` | CMakeLists.txt and package.xml already correct |

---

## Architecture Patterns

### Provisioning Order (strictly sequential)

```
1. System prep: locale, apt update, add ROS2 apt source
2. Install: ros-humble-ros-base, ros-dev-tools
3. Install: rustup (curl | sh), source ~/.cargo/env
4. Install: libclang-dev cmake pkg-config libudev-dev build-essential
5. Build polaris_msgs: mkdir ~/ros2_ws/src, symlink/copy, colcon build
6. Source both environments: /opt/ros/humble/setup.bash + ~/ros2_ws/install/setup.bash
7. cargo build --release for polaris crates
```

Each step is a hard dependency on the previous. r2r's build.rs inspects the sourced ROS2 environment at compile time — step 6 must happen in the same shell as step 7.

### Pattern: Source order matters for r2r

r2r discovers message packages by inspecting the currently sourced ROS2 environment. The build produces correct Rust types only if all needed packages are visible.

```bash
# Correct order in any shell that runs cargo build
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # polaris_msgs lives here
cargo build --release
```

If `polaris_msgs` is not sourced, `r2r::polaris_msgs::msg::VehicleState` will not exist and polaris_bridge will fail to compile.

### Pattern: IDL_PACKAGE_FILTER in .cargo/config.toml

The project already has this configured. Without it, r2r would compile bindings for every message package in the workspace, significantly increasing build time. The filter is at `.cargo/config.toml`:

```toml
[env]
IDL_PACKAGE_FILTER = "polaris_msgs;std_msgs;std_srvs;sensor_msgs;geometry_msgs;builtin_interfaces"
```

This is a workspace-level `.cargo/config.toml` — it applies to all crates in the workspace automatically. No per-crate changes needed.

### Pattern: polaris_msgs colcon workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Copy or symlink polaris_msgs here
cp -r /path/to/polaris/polaris_msgs .
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select polaris_msgs
source install/setup.bash
```

After this, `ros2 interface show polaris_msgs/msg/VehicleState` should return the message definition.

### Pattern: Forcing r2r message type rebuild

If polaris_msgs changes after initial build:
```bash
cargo clean -p r2r_msg_gen
cargo build --release
```

This forces r2r's message generation crate to re-run, picking up the updated message definitions.

### Anti-Patterns to Avoid

- **Installing ros-humble-desktop:** Pulls `libopencv-dev` from Ubuntu apt, which conflicts with JetPack's pre-installed OpenCV. Use `ros-humble-ros-base` instead.
- **Running cargo build without sourcing ROS2:** r2r will either fail to find RCL libraries or link against a stale environment. Must source in the same shell before building.
- **Sourcing workspace but not base:** `~/ros2_ws/install/setup.bash` chains from `/opt/ros/humble/setup.bash`, but sourcing only the workspace without first sourcing the base can cause incomplete environment.
- **Installing Rust via apt:** `apt install rustc` gives an old system Rust (often 1.65 or older). Always use rustup for a current stable toolchain.
- **Using nohup for nodes without forwarding LD_LIBRARY_PATH:** Known pitfall from project memory — when launching nodes with nohup, ROS2 shared libraries become invisible. Must explicitly pass `LD_LIBRARY_PATH=$LD_LIBRARY_PATH`.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| ROS2 package discovery | Custom pkg-config wrapper | r2r's IDL_PACKAGE_FILTER | r2r already handles this; .cargo/config.toml already has it |
| Message type generation | Custom bindgen setup | r2r's build.rs | r2r introspects C libraries, handles all ROS2 message types |
| colcon workspace setup | Custom CMake scripts | Standard colcon with ament_cmake | polaris_msgs/CMakeLists.txt already correct |

**Key insight:** r2r's value proposition is precisely that it requires no custom ROS2 build infra — just source the environment and run `cargo build`. Every workaround that bypasses this pattern increases maintenance cost.

---

## Common Pitfalls

### Pitfall 1: JetPack OpenCV conflict
**What goes wrong:** `sudo apt install ros-humble-desktop` fails or creates broken state because JetPack pre-installs OpenCV 4.5.x and `ros-humble-desktop` tries to install `libopencv-dev` from Ubuntu apt, causing dpkg conflicts.
**Why it happens:** JetPack ships its own CUDA-optimized OpenCV; Ubuntu's ROS-dependent OpenCV is a different build.
**How to avoid:** Use `ros-humble-ros-base` instead of `ros-humble-desktop`. All polaris crates need only the base (no RViz, no GUI).
**Warning signs:** `dpkg: error processing package libopencv-dev` or `files list file for package 'libopencv4.5-dev' is missing final newline`

### Pitfall 2: Rust toolchain too old
**What goes wrong:** `cargo build` fails with syntax errors or missing features because `rustc` is from apt (may be 1.65 or older on Ubuntu 22.04).
**Why it happens:** Ubuntu 22.04's packaged `rustc` is significantly older than current stable.
**How to avoid:** Install via `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`, then `source ~/.cargo/env`. Verify with `rustc --version` (need 1.75+).
**Warning signs:** `error[E0XXX]` on valid Rust syntax, or `rustc --version` showing < 1.75.

### Pitfall 3: cargo build without sourced ROS2
**What goes wrong:** r2r fails to find RCL headers/libraries, or links against a different ROS2 install, or `polaris_msgs` types are missing.
**Why it happens:** r2r's build.rs uses `ament_cmake` environment variables set by `setup.bash` to locate ROS2 installations.
**How to avoid:** Always `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash` in the same shell before running `cargo build`.
**Warning signs:** `error: failed to run custom build command for r2r` or `error[E0412]: cannot find type VehicleState in module r2r::polaris_msgs::msg`

### Pitfall 4: libclang-dev missing
**What goes wrong:** r2r's build fails at the bindgen step with a clang-related error.
**Why it happens:** bindgen (used by r2r internally) requires libclang to parse C headers.
**How to avoid:** `sudo apt install libclang-dev` before first `cargo build`. This is documented explicitly in the r2r README.
**Warning signs:** `error: failed to find libclang` or `Unable to find libclang`

### Pitfall 5: polaris_msgs not in ros2_ws before colcon build
**What goes wrong:** `r2r::polaris_msgs` module doesn't exist at compile time; polaris_bridge fails to build.
**Why it happens:** r2r reads the generated C introspection code from the colcon-installed package. The code is only generated after `colcon build`.
**How to avoid:** Build polaris_msgs first (`colcon build --packages-select polaris_msgs` in `~/ros2_ws/`), then source `~/ros2_ws/install/setup.bash` before cargo build.
**Warning signs:** `error[E0433]: failed to resolve: use of undeclared crate or module polaris_msgs`

### Pitfall 6: nohup loses LD_LIBRARY_PATH
**What goes wrong:** Nodes started with nohup fail at runtime with `librcl_action.so: cannot open shared object file`.
**Why it happens:** nohup doesn't inherit shell-sourced env vars. ROS2 shared library paths set by `setup.bash` are lost.
**How to avoid:** When using nohup: `nohup env LD_LIBRARY_PATH=$LD_LIBRARY_PATH ./target/release/node &`
**Warning signs:** Binary works interactively but fails when launched non-interactively.

---

## Code Examples

### ROS2 Humble Install (aarch64 Ubuntu 22.04)
```bash
# Source: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade
sudo apt install ros-humble-ros-base ros-dev-tools
```

### Rust via rustup
```bash
# Source: https://rustup.rs/
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain stable -y
source ~/.cargo/env
rustc --version  # Should show 1.75+
```

### Build Dependencies
```bash
sudo apt install libclang-dev cmake pkg-config libudev-dev build-essential
```

### polaris_msgs colcon workspace
```bash
mkdir -p ~/ros2_ws/src
cp -r ~/polaris/polaris_msgs ~/ros2_ws/src/
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select polaris_msgs
source install/setup.bash
# Verify:
ros2 interface show polaris_msgs/msg/VehicleState
```

### cargo build for polaris crates
```bash
# Must be in same shell where both setups are sourced
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/polaris
cargo build --release -p polaris_bridge
cargo build --release -p polaris_operator
cargo build --release -p polaris_control
```

### Verify ROS2 installation
```bash
source /opt/ros/humble/setup.bash
ros2 topic list     # Should return /parameter_events and /rosout
ros2 topic echo /input/steering --once  # After a node is running
```

### Force r2r message rebuild (after polaris_msgs changes)
```bash
cargo clean -p r2r_msg_gen
cargo build --release
```

---

## State of the Art

| Old Approach | Current Approach | Impact |
|--------------|------------------|--------|
| Install Rust via `apt install rustc` | Install via rustup | apt gives 1.65 on Ubuntu 22.04; rustup gives current stable (1.82+) |
| Manual GPG key + sources.list | ros2-apt-source deb package | New recommended method as of 2024; single .deb handles key + repo |
| colcon installed separately | Included in `ros-dev-tools` | ros-dev-tools meta-package simplifies install |

---

## Open Questions

1. **Does the Jetson have JetPack installed, and if so which version?**
   - What we know: The machine reports `Linux 5.15.136-tegra` (kernel), Ubuntu 22.04 Jammy. The `-tegra` suffix indicates L4T (Linux for Tegra) — JetPack is present.
   - What's unclear: JetPack 5.x (L4T 35.x) vs JetPack 6.x (L4T 36.x) determines whether NVIDIA's Isaac apt mirror is an option, but this is irrelevant since we're using upstream ROS2 deb packages.
   - Recommendation: Use standard ros.org apt source regardless. The `ros-humble-ros-base` path avoids all JetPack-specific OpenCV issues.

2. **Will polaris_operator compile on Jetson (gilrs gamepad)?**
   - What we know: gilrs requires `libudev-dev` on Linux. The Jetson runs standard Linux with udev.
   - What's unclear: gilrs enumerates `/dev/input/event*` — the Jetson's input subsystem should work, but there's no joystick connected. Build should succeed; runtime would require a connected device.
   - Recommendation: Include `libudev-dev` in build deps. Build will succeed even without attached gamepad.

3. **polaris_wheel: is it in scope for this phase?**
   - What we know: polaris_wheel is listed in workspace Cargo.toml but per SPEC.md it's a Mac-only binary (SDL2 + QUIC, no ROS2). It doesn't use r2r.
   - What's unclear: PROV-05 says "polaris_bridge, polaris_operator, polaris_control" — polaris_wheel is not listed.
   - Recommendation: Exclude polaris_wheel from this phase's build verification. It requires SDL2 which is unnecessary on Jetson.

---

## Sources

### Primary (HIGH confidence)
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html — exact ROS2 Humble apt install commands, ros-humble-ros-base vs desktop distinction
- https://github.com/sequenceplanner/r2r — IDL_PACKAGE_FILTER documentation, libclang-dev requirement, source-before-build requirement, `cargo clean -p r2r_msg_gen`
- https://rustup.rs / https://rust-lang.org/tools/install — rustup install command
- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html — colcon workspace setup and sourcing

### Secondary (MEDIUM confidence)
- https://github.com/m-dahl/r2r_minimal_node — confirmed IDL_PACKAGE_FILTER in .cargo/config.toml pattern
- https://github.com/serialport/serialport-rs — libudev-dev requirement for serialport (mavlink direct-serial)
- https://lib.rs/crates/gilrs-core — libudev-dev / pkg-config requirement for gilrs on Linux
- https://forums.developer.nvidia.com (multiple threads) — JetPack OpenCV conflict with ros-humble-desktop confirmed by community reports

### Tertiary (LOW confidence — from project memory, already experienced)
- Project MEMORY.md: nohup + LD_LIBRARY_PATH pitfall (experienced directly on fastgpu)
- Project MEMORY.md: `cargo clean -p r2r_msg_gen` for stale .so after message changes

---

## Metadata

**Confidence breakdown:**
- ROS2 Humble install on Ubuntu 22.04 aarch64: HIGH — official docs, aarch64 is tier-1 supported
- Rust via rustup on aarch64: HIGH — official installer handles detection automatically
- r2r build requirements: HIGH — verified from r2r GitHub README directly
- IDL_PACKAGE_FILTER usage: HIGH — confirmed in r2r README + r2r_minimal_node example
- polaris_msgs colcon workflow: HIGH — standard ament_cmake rosidl pattern, CMakeLists.txt already correct
- JetPack OpenCV conflict: MEDIUM — confirmed by community reports, not NVIDIA official docs
- gilrs libudev-dev requirement: HIGH — explicitly documented in gilrs-core

**Research date:** 2026-02-23
**Valid until:** 2026-05-23 (90 days — ROS2 Humble is stable/LTS, Rust stable install process is stable)
