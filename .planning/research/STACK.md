# STACK — Jetson Table Rig

**Research Date:** 2026-02-23
**Scope:** Deploying existing Rust/ROS2 codebase on Jetson NRU-160 (Ubuntu 22.04, aarch64) with MAVLink over Ethernet to a Pixhawk. Jetson-specific tooling only — core ROS2/Rust stack not re-researched.

---

## Operating Environment

**Target:** Jetson NRU-160, Ubuntu 22.04 Jammy, Linux 5.15.136-tegra, aarch64
**Context:** JetPack 6.x ships Ubuntu 22.04 + kernel 5.15 on all Jetson Orin variants. The NRU-160 is an Orin-class module. The existing OS is vanilla Ubuntu 22.04 (no JetPack CUDA stack required for this use case — we're doing control, not inference).

---

## Layer 1: ROS2

**Choice:** `ros-humble-ros-base` via apt

**Why Humble, not Jazzy:**
Ubuntu 22.04 Jammy only has apt packages for ROS2 Humble. Jazzy targets Ubuntu 24.04 (Noble). Installing Jazzy on 22.04 requires source compilation, which is fragile and unnecessary. Humble's EOL is May 2027 — well past this milestone's horizon.

**Why ros-base, not desktop:**
`ros-humble-desktop` adds RViz, rqt, and GUI tools — useless on a headless embedded board. `ros-base` installs exactly what r2r needs: rcl, rclcpp, rmw, DDS, and all standard message packages. Saves ~500MB.

**Installation (verified from official docs):**

```bash
# Locale
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

# ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# Install
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

The `$(dpkg --print-architecture)` macro resolves to `arm64` automatically on the Jetson — no architecture-specific modifications needed.

**Confidence:** High. ROS2 Humble is a Tier 1 platform on Ubuntu 22.04 aarch64 — official binary packages exist for this exact combination.

**What NOT to use:**
- RoboStack (conda-based ROS2): adds conda layer with no benefit over apt; apt is simpler and has better library compatibility with system packages
- Source build of Humble: weeks of dependency pain, no advantage here
- Jazzy: no Ubuntu 22.04 apt packages; would require source build

---

## Layer 2: Custom Message Package (polaris_msgs)

**Choice:** colcon build, run once on Jetson, output stays in `~/ros2_ws/install/`

**Why:** r2r's build.rs script links against the installed message headers from the colcon workspace. This is a one-time provisioning step. The colcon workspace on the Jetson is structurally identical to the one on fastgpu — same `polaris_msgs/CMakeLists.txt` and `package.xml`, same `colcon build --packages-select polaris_msgs` command, just sourcing `/opt/ros/humble/` instead of `/opt/ros/jazzy/`.

**Required apt packages (beyond ros-base):**

```bash
sudo apt install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-ament-cmake \
  python3-colcon-common-extensions
```

`rosidl-default-generators` and `ament-cmake` are build-time dependencies for `.msg` generation. They are pulled in transitively by `ros-humble-ros-base` in most configurations but explicit installation avoids colcon errors on minimal installs.

**Build command (run once on Jetson):**

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cp -r ~/polaris/polaris_msgs ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select polaris_msgs
```

After this, `source ~/ros2_ws/install/setup.bash` makes the VehicleState message headers available to r2r's build.rs.

**Confidence:** High. This exact workflow is tested on fastgpu (Jazzy). ROS2 Humble uses the same colcon/rosidl pipeline — the only difference is the setup.bash path.

---

## Layer 3: Rust Toolchain

**Choice:** rustup stable, installed natively on Jetson

**Why native compile (not cross-compile):**
Cross-compiling Rust + r2r + colcon for aarch64 is fragile. r2r's build.rs uses libclang (via bindgen) to introspect ROS2 C headers at compile time — getting the right sysroot, header paths, and library paths in a cross-compile environment adds significant complexity. The Jetson has 16GB RAM and 202GB NVMe. A `cargo build --release` of the five crates takes 5–10 minutes natively on Orin-class hardware (Cortex-A78AE at 2.2GHz). That's acceptable for a bench rig with infrequent rebuilds.

**Installation:**

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
rustup default stable
```

This installs the `aarch64-unknown-linux-gnu` toolchain natively. No target additions needed — the host is already aarch64.

**Version:** Rust stable. The project currently runs Rust 1.93.0 on fastgpu. Stable on the Jetson will be the same or newer — r2r 0.9.5 has no MSRV beyond "recent stable".

**What NOT to use:**
- `apt install rustc`: Ubuntu 22.04's apt package is Rust 1.66 — too old for the project's edition/features. rustup always installs current stable.
- Cross-compilation via `cross` or `cargo-cross`: viable for CI, fragile for r2r (bindgen needs host libclang to know about target headers); not worth the complexity for a bench rig with 16GB RAM.

**Confidence:** High. Rust natively compiles on aarch64-unknown-linux-gnu without modification. This target is Tier 1 in the Rust toolchain.

---

## Layer 4: r2r Build Dependencies

**Choice:** `libclang-dev` (for bindgen) + `cmake` (for colcon)

r2r uses bindgen internally to generate Rust FFI bindings from ROS2 C headers at `cargo build` time. bindgen requires libclang at build time (not runtime).

```bash
sudo apt install -y libclang-dev cmake pkg-config
```

**Why these:**
- `libclang-dev`: bindgen's hard dependency; without it, r2r's build.rs fails with "clang not found"
- `cmake`: required by colcon for building polaris_msgs (ament_cmake)
- `pkg-config`: needed by several Cargo build scripts to locate system libraries

**IDL_PACKAGE_FILTER:** The existing `.cargo/config.toml` already sets `IDL_PACKAGE_FILTER = "polaris_msgs;std_msgs;std_srvs;sensor_msgs;geometry_msgs;builtin_interfaces"`. This limits r2r bindgen to only the packages actually used — critical on the Jetson because generating bindings for all 200+ ROS2 message packages would balloon compile time.

**Confidence:** High. Documented in r2r README and confirmed by community build reports on arm64.

---

## Layer 5: MAVLink over Ethernet

**Choice:** `mavlink` crate 0.14.1 (already in Cargo.toml), connection string `udpout:192.168.88.242:14550`

**How it works:**
The `mavlink` crate's `connect()` function parses a connection string and returns a `MavConnection` handle. The string format for UDP is `udpout:<host>:<port>` (Jetson initiates, sends packets to Pixhawk) or `udpin:<bind-addr>:<port>` (Jetson listens, Pixhawk initiates).

**Which mode to use:**
ArduPilot's native Ethernet implementation (NET_P1 framework, ArduPilot 4.5+) can act as either UDP client or server. The safer default is `udpout:192.168.88.242:14550` — the Jetson sends to the Pixhawk's well-known address. This requires the Pixhawk to be configured as a UDP server (NET_P1_TYPE=2, NET_P1_PORT=14550). If the Pixhawk is configured as a UDP client instead, use `udpin:0.0.0.0:14550` to listen.

**The env var change (only code-adjacent change needed):**

```bash
# In launch/rig.sh
PIXHAWK_ADDR=udpout:192.168.88.242:14550
```

No source code changes — the bridge already reads `PIXHAWK_ADDR` from the environment.

**Pixhawk configuration (ArduPilot parameters to set once via Mission Planner or MAVProxy):**
```
NET_ENABLE = 1         # Enable Ethernet (requires reboot)
NET_P1_TYPE = 2        # UDP Server
NET_P1_PORT = 14550    # Listen port
NET_P1_PROTOCOL = 2    # MAVLink2
```
After reboot, the Pixhawk listens for MAVLink packets on UDP port 14550.

**Note on port:** Port 14550 is the de-facto standard for ground station connections (GCS profile). Port 14540 is for offboard APIs (Onboard profile, which broadcasts a smaller message set). Either works with the `ardupilotmega` dialect — 14550 is more permissive and confirmed working in ArduPilot community setups.

**What NOT to use:**
- Serial (`/dev/ttyUSB0`): The Pixhawk is at 192.168.88.242 — no serial cable in this setup
- TCP: UDP is lower latency and avoids connection state; use TCP (`tcpout:192.168.88.242:5760`) only as fallback if UDP proves unreliable
- `udpbcast`: broadcast MAVLink works but is noisy on the shared Teltonika network; unicast to the known IP is cleaner

**Confidence:** Medium-high. The `udpout`/`udpin` connection strings are well-documented in the rust-mavlink crate (docs.rs/mavlink). The ArduPilot NET_P1 framework is documented for Pixhawk6X and similar Ethernet-capable flight controllers. The NRU-160's Pixhawk variant may have a different port default — verify with `mavproxy.py --master=udpin:0.0.0.0:14550 --target-system=1` before writing the launch script.

---

## Layer 6: QUIC (Quinn) — No Changes Needed

**Choice:** `quinn` 0.11.9 (unchanged from fastgpu)

The QUIC server (`wheel_bridge_node`) binds to `0.0.0.0:9876` on any interface. On the Jetson it will bind to the Ethernet interface's address (192.168.88.175:9876). The Teltonika RUTM51 forwards UDP by default — no port forwarding needed since all devices are on the same 192.168.88.x LAN.

The TLS cert generation (rcgen, first-run creates `~/.config/polaris/cert.der` + `key.der`) works identically on Linux aarch64.

One minor code change: `polaris_wheel` on Mac currently uses a hardcoded or default server address pointing to fastgpu. This needs to become a CLI argument or `POLARIS_SERVER` env var to point at 192.168.88.175:9876. This is already identified in the ARCHITECTURE research.

**Confidence:** High. quinn has no platform-specific code; it runs on any Rust-supported target.

---

## Layer 7: Sync and Build Tooling

**Choice:** `scp -r` push from Mac → Jetson (same pattern as fastgpu sync.sh)

A new `launch/rig.sh` mirrors `launch/sim.sh` with:
- `scp -r` to `nvidia@192.168.88.175:~/polaris/`
- Build command: `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cargo build --release -p polaris_bridge -p polaris_operator -p polaris_control`
- Note: `polaris_sim` is excluded — BeamNG bridge has no role on the Jetson
- `nohup` launch with `LD_LIBRARY_PATH=$LD_LIBRARY_PATH` explicitly passed (same pattern as fastgpu from project memory — `nohup` drops env vars)

**Binary paths:** `~/polaris/target/release/{pixhawk_bridge_node,vehicle_control_node,wheel_bridge_node,tui_dashboard}`

**Confidence:** High. Same pattern already proven on fastgpu. Only host address and ROS2 path change.

---

## Full Provisioning Sequence

Run once on a fresh Jetson (SSH session):

```bash
# 1. ROS2 Humble
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions \
  ros-humble-rosidl-default-generators ros-humble-ament-cmake

# 2. r2r build deps
sudo apt install -y libclang-dev cmake pkg-config

# 3. Rust toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source ~/.cargo/env

# 4. polaris_msgs workspace (run after source is synced)
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cp -r ~/polaris/polaris_msgs ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select polaris_msgs
```

After provisioning, subsequent deploys are: sync source → `cargo build --release` → restart processes.

---

## Version Summary

| Component | Version | Source | Confidence |
|-----------|---------|--------|------------|
| Ubuntu | 22.04 Jammy | Pre-installed | Confirmed |
| ROS2 | Humble (EOL May 2027) | apt (ros.org) | High |
| Rust | stable (≥1.93.0) | rustup | High |
| r2r | 0.9.5 | Cargo.toml (unchanged) | High |
| mavlink crate | 0.14.1 | Cargo.toml (unchanged) | High |
| quinn | 0.11.9 | Cargo.toml (unchanged) | High |
| libclang-dev | apt (Ubuntu 22.04) | apt | High |
| MAVLink connection | `udpout:192.168.88.242:14550` | env var | Medium-High |
| Pixhawk UDP port | 14550 | ArduPilot default | Medium |

---

## What NOT to Do (and Why)

| Approach | Rejected Because |
|----------|-----------------|
| ROS2 Jazzy on Ubuntu 22.04 | No apt packages; source build is weeks of work with no benefit |
| ros-humble-desktop | Includes GUI tools (RViz, rqt) useless on headless Jetson; +500MB |
| Cross-compile from Mac | bindgen in r2r's build.rs needs host libclang to know target headers; fragile setup for 10-min native compile |
| apt rustc | Version 1.66 — too old; project requires recent stable |
| RoboStack / conda ROS2 | Extra abstraction layer, worse library compatibility, no advantage over apt |
| UDP broadcast MAVLink | Noisy on shared Teltonika LAN; unicast to known IP is cleaner |
| TCP MAVLink as primary | Higher latency than UDP; use only as fallback |

---

Sources:
- [Ubuntu (deb packages) — ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [connect in mavlink — Rust docs.rs](https://docs.rs/mavlink/latest/mavlink/fn.connect.html)
- [GitHub — sequenceplanner/r2r: Minimal ROS 2 Rust bindings](https://github.com/sequenceplanner/r2r)
- [Ethernet / Network Setup — ArduPilot Copter documentation](https://ardupilot.org/copter/docs/common-network.html)
- [rustup.rs — The Rust toolchain installer](https://rustup.rs/)
- [Jetson AGX Orin Rust Compatibility — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/jetson-agx-orin-rust-compatibility/235653)
- [JetPack 6.2.2/Jetson Linux 36.5 — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/jetpack-6-2-2-jetson-linux-36-5-is-now-live/359622)
