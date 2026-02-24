```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           POLARIS SYSTEM MAP                                │
└─────────────────────────────────────────────────────────────────────────────┘

  MAC (operator laptop)                        FASTGPU (all ROS2 nodes)
 ─────────────────────────────────────────  ──────────────────────────────────

 ┌──────────────┐
 │  G923 Wheel  │ (USB hardware)
 │  pedals +    │
 │  paddles     │
 └──────┬───────┘
        │ SDL2 joystick (polaris_wheel)
        ▼
 ┌──────────────┐                            ┌──────────────────┐
 │polaris_wheel │────QUIC:9876─────────────▶│ wheel_bridge     │
 │  (binary)    │    14-byte cmd packets     │ _node            │
 │              │                            │                  │
 │  ratatui TUI │◀───QUIC:9876──────────────┤  pub /input/*    │
 │  (telemetry) │    72-byte telemetry       │  sub /vehicle/*  │
 └──────────────┘                            │  sub /cmd/*      │
                                             └────────┬─────────┘
                                                      │
                                                      │ /input/* (50Hz)
                                                      ▼
 ┌──────────────┐  (SSH to fastgpu)          ┌──────────────────┐
 │tui_dashboard │───keyboard WASD──────────▶│ vehicle_control  │
 │  (over SSH)  │   pub /input/* (30Hz)      │ _node            │
 │              │                            │                  │
 │  ┌────────┐  │   /vehicle/state           │  passthrough:    │
 │  │ display│◀─┼───────────────────────────┤  /input/* → /cmd/*│
 │  └────────┘  │                            │  at 50Hz         │
 └──────────────┘                            └────────┬─────────┘
                                                      │
                                                      │ /cmd/* (50Hz)
                                                      ▼
                                          ┌─────────────────────────┐
                                          │                         │
                               ┌──────────────┐         ┌──────────────┐
                               │   pixhawk    │   OR    │   beamng     │
                               │   bridge     │         │   bridge     │
                               │   (node)     │         │   (node)     │
                               │              │         │              │
                               │  MAVLink     │         │  TCP+msgpack │
                               │  serial/TCP  │         │  to BeamNG   │
                               └──────┬───────┘         └──────┬───────┘
                                      │                        │
                            ┌─────────┼─────────┐              │
                            ▼                   ▼              ▼
                     ┌──────────────┐  ┌──────────────┐ ┌──────────────┐
                     │   Pixhawk    │  │ ArduPilot    │ │   BeamNG     │
                     │  (hardware)  │  │ SITL (TCP)   │ │  (simulator) │
                     │              │  │              │ │              │
                     │  serial      │  │  virtual     │ │              │
                     │  5 actuators │  │  sim physics  │ │              │
                     └──────────────┘  └──────────────┘ └──────────────┘
                      real vehicle      simulated rig       BeamNG sim


 ─────────────────────────────────────────────────────────────────────────────
  TOPICS
 ─────────────────────────────────────────────────────────────────────────────

  Topic              Msg Type                  Dir     Who pub              Who sub
  ─────              ────────                  ───     ───────              ───────
  /input/steering    Float32                   →       wheel_bridge/tui     vehicle_control
  /input/throttle    Float32                   →       wheel_bridge/tui     vehicle_control
  /input/brake       Float32                   →       wheel_bridge/tui     vehicle_control
  /input/gear        UInt8                     →       wheel_bridge/tui     vehicle_control
  /input/handbrake   Bool                      →       wheel_bridge/tui     vehicle_control
  /cmd/steering      Float32                   →       vehicle_control      bridge
  /cmd/throttle      Float32                   →       vehicle_control      bridge
  /cmd/brake         Float32                   →       vehicle_control      bridge
  /cmd/gear          UInt8                     →       vehicle_control      bridge
  /cmd/handbrake     Bool                      →       vehicle_control      bridge
  /vehicle/state     VehicleState              ←       bridge               all
  /vehicle/imu       sensor_msgs/Imu           ←       pixhawk_bridge       tui, wheel_bridge
  /vehicle/gps       sensor_msgs/NavSatFix     ←       pixhawk_bridge       tui, wheel_bridge


 ─────────────────────────────────────────────────────────────────────────────
  FILES MAP (by crate, with key lines)
 ─────────────────────────────────────────────────────────────────────────────

  polaris_wheel/            ← BUILDS ON MAC (standalone, no ROS2)
    main.rs
      :12   MAX_STEER = 7.854 rad (450 degrees)
      :16   AXIS_STEERING/THROTTLE/BRAKE (SDL2 axis indices)
      :22   BTN_RIGHT_PADDLE/LEFT_PADDLE/CIRCLE/OPTIONS (SDL2 button indices)
      :28   WheelPacket struct — 14-byte command packet
      :48   VehicleDown struct — 72-byte telemetry packet
      :139  pedal_to_normalized() — SDL2 axis → 0..1
      :144  steering_to_normalized() — SDL2 axis → -1..1 with deadzone
      :250  target = CLI arg or "fastgpu.local:9876"
      :188  QUIC task: auto-reconnect, bidirectional stream (14B up, 72B down at 50Hz)
      :228  SDL2 event pump + button edge detection (gear, handbrake, e-stop)
      :294  draw() — ratatui TUI (header, connections, actuators, telemetry)

  polaris_operator/           ← BUILDS ON FASTGPU
    gamepad.rs                reads G923 via gilrs (shared module)
      :5    GamepadState struct — steering/throttle/brake/gear/handbrake + edge state
      :20   new(max_steer, deadzone)
      :36   poll() — read gilrs events + axes
      :47   Right/Left trigger → gear shift (edge-triggered)
      :69   East button → handbrake toggle
      :80   Start button → e-stop
      :92   LeftStickX → steering, LeftStickY → throttle, LeftZ → brake

    main.rs                   tui_dashboard / tui_monitor (--monitor flag)
      :59   STEER_INCREMENT=0.05, STEER_DECAY=0.8, MAX_STEER=7.854
      :65   DashboardState — all display state
      :113  monitor_mode = --monitor flag
      :128  GamepadState::new(MAX_STEER, 0.05)
      :141  Gamepad poll — gilrs takes priority over keyboard
      :159  Keyboard input: W=throttle, S=brake, A/D=steer, Space=stop
      :184  Decay: steering*0.8, throttle/brake -0.05/tick (keyboard only)
      :201  draw() — header, connections, actuators, telemetry, footer
      :291  node name: "tui_dashboard" or "tui_monitor"
      :297  sub /vehicle/state (sensor QoS)
      :301  sub /cmd/* (safe actuator values)
      :308  sub /input/* (operator input, for monitor display)
      :366  sub /vehicle/imu
      :380  sub /vehicle/gps
      :435  pub /input/* at 30Hz (only when local gamepad connected)

    bin/gamepad_node.rs       standalone headless gamepad publisher
      :10   node name: "gamepad"
      :13   pub /input/* (5 topics)
      :26   GamepadState::new(0.5236, 0.05) — 30 degree max steer
      :31   50Hz main loop

    bin/wheel_bridge_node.rs  QUIC:9876 server ↔ ROS2
      :7    LISTEN_PORT = 9876
      :9    WheelState struct — 14-byte command packet layout
      :23   parse_packet() — 14 bytes → WheelState
      :34   VehicleDown struct — 72-byte telemetry packet layout
      :52   to_bytes() — VehicleDown → 72 bytes
      :80   node name: "wheel_bridge"
      :83   pub /input/* (5 topics)
      :92   sub /vehicle/state
      :106  sub /cmd/* (safe actuator values → telemetry downstream)
      :127  sub /vehicle/imu
      :141  sub /vehicle/gps
      :156  QUIC endpoint on 0.0.0.0:9876
      :166  Writer thread: 72-byte packets at 30Hz
      :178  Reader thread: 14-byte command packets
      :210  ROS2 publish loop: 50Hz

  polaris_control/          ← BUILDS ON FASTGPU
    main.rs                   vehicle_control_node (passthrough)
      :5    Actuators struct — steering/throttle/brake/gear/handbrake
      :28   node name: "vehicle_control"
      :34   sub /input/steering, /input/throttle, /input/brake (Float32)
      :37   sub /input/gear (UInt8)
      :38   sub /input/handbrake (Bool)
      :41   pub /cmd/steering, throttle, brake (Float32)
      :44   pub /cmd/gear (UInt8), /cmd/handbrake (Bool)
      :47   50Hz timer (20ms)
      :86   Forward loop: read Actuators, publish to /cmd/*

  polaris_bridge/           ← BUILDS ON FASTGPU (real vehicle + SITL)
    pixhawk.rs                MAVLink serial/TCP interface (no ROS2)
      :5    GEAR_PWM = [1000, 1250, 1500, 1750] — P, R, N, D
      :6    HANDBRAKE_ENGAGED = 2000, HANDBRAKE_RELEASED = 1000
      :20   PixhawkInterface struct — wraps MavConnection
      :27   connect() — serial or TCP, wait heartbeat (30s)
      :53   set_manual_mode() — MAV_CMD_DO_SET_MODE
      :67   send_neutral_rc() — center steering, brake off
      :81   arm() — force arm (param2=21196)
      :95   request_data_streams() — MAV_DATA_STREAM_ALL
      :106  send_rc_override() — 5 channels:
             ch1=steering (centered PWM), ch2=brake, ch3=throttle,
             ch4=gear (discrete PWM), ch5=handbrake (binary)
      :127  recv_telemetry() → TelemetryUpdate::Position or ::Imu
      :148  TelemetryUpdate enum — Position{lat,lon,alt,heading,speed}, Imu{acc,gyro}
      :153  value_to_pwm(), value_to_pwm_centered() — float → PWM mapping

    main.rs                   pixhawk_bridge_node
      :31   node name: "pixhawk_bridge"
      :33   PIXHAWK_ADDR env var (default: serial:/dev/ttyUSB0:921600)
      :34   max_steering_angle = 7.854 (450 degrees)
      :41   sub /cmd/* (5 topics, reliable QoS)
      :47   pub /vehicle/state (VehicleState, sensor QoS)
      :48   pub /vehicle/imu (sensor_msgs/Imu, sensor QoS)
      :49   pub /vehicle/gps (sensor_msgs/NavSatFix, sensor QoS)
      :51   Connection retry timer: 2s
      :87   connect() → request_data_streams(10Hz) → set_manual_mode() → arm()
      :104  Telemetry reader thread (blocking MAVLink recv in std::thread)
      :115  send_rc_override on every telemetry message (piggybacks on recv rate)

  polaris_sim/              ← BUILDS ON FASTGPU (BeamNG simulation)
    bridge.rs                 BeamNG TCP+msgpack client (no ROS2)
      :6    PROTOCOL_VERSION = "v1.26"
      :8    ConnectionState enum — Disconnected/Connected/Ready/Error
      :16   BeamNGBridge struct — two TCP streams (ge + veh), control state
      :58   connect() — TCP to host:port, Hello handshake
      :81   attach_vehicle() — GetCurrentVehicles → StartVehicleConnection
      :120  Disable player input via Lua
      :131  send_control() — normalize steering by max_steering_angle
      :148  poll_state() — SensorRequest → parse pos/vel/rotation
      :191  VehiclePhysics struct — pos[3], vel[3], rot[4]
      :198  Wire: [u32 BE length][msgpack map], auto-incrementing _id

    main.rs                   beamng_bridge_node
      :26   node name: "beamng_bridge"
      :28   host = "fastgpu", port = 64256
      :30   max_steering_angle = 0.5236 (30 degrees)
      :36   sub /cmd/* (5 topics)
      :42   pub /vehicle/state (VehicleState, sensor QoS)
      :70   Bridge loop: connect → attach → send_control → poll_state → publish
      :107  Speed from velocity magnitude, heading from quaternion
      :134  20ms sleep (50Hz control + publish loop)

  polaris_msgs/             ← BUILDS EVERYWHERE (colcon)
    msg/VehicleState.msg
      :1    float32 steering_angle, throttle_position, brake_pressure
      :4    uint8 gear_current (0=P, 1=R, 2=N, 3=D)
      :5    bool handbrake_engaged
      :6    float32 speed_mps, heading_deg
      :8    float64 lat, lon
      :10   uint8 mode (0=MANUAL, 1=REMOTE, 2=AUTO)
      :11   builtin_interfaces/Time stamp


┌─────────────────────────────────────────────────────────────────────────────┐
│                         NODES & BINARIES                                    │
└─────────────────────────────────────────────────────────────────────────────┘

  Binary                Crate             Node Name         Where
  ──────                ─────             ─────────         ─────
  polaris_wheel         polaris_wheel     (no ROS2)         Mac
  tui_dashboard         polaris_operator    tui_dashboard     fastgpu (over SSH)
  tui_dashboard --mon   polaris_operator    tui_monitor       fastgpu (HDMI)
  gamepad_node          polaris_operator    gamepad           fastgpu
  wheel_bridge_node     polaris_operator    wheel_bridge      fastgpu
  vehicle_control_node  polaris_control   vehicle_control   fastgpu
  pixhawk_bridge_node   polaris_bridge    pixhawk_bridge    fastgpu
  beamng_bridge_node    polaris_sim       beamng_bridge     fastgpu


  DATA FLOW (end to end)
 ─────────────────────────────────────────────────────────────────────────────

  G923 wheel (Mac USB)
       │ SDL2 joystick
       ▼
  polaris_wheel (Mac binary, ratatui TUI)
       │ QUIC:9876 (14-byte packets at 50Hz)
       ▼
  ┌──────────┐  /input/*    ┌─────────────────┐  /cmd/*       ┌────────────┐
  │ wheel    │─────────────▶│ vehicle_control │───────────────▶│  pixhawk   │──▶ SITL / Pixhawk
  │ _bridge  │   50Hz       │ _node           │   50Hz         │  _bridge   │
  └──────────┘               │                 │                │            │
                             │ passthrough:    │                │  MAVLink   │
                             │  stores latest  │                │  RC over-  │
                             │  /input/*, re-  │                │  ride +    │
                             │  publishes on   │                │  telemetry │
                             │  50Hz timer     │                └─────┬──────┘
                             └─────────────────┘                      │
                                                                      │
  polaris_wheel (Mac) ◀── QUIC:9876 (72-byte telemetry at 30Hz)      │
                                                                      │
  tui_dashboard ◀────── /vehicle/state + /vehicle/imu + /vehicle/gps ─┘


  MSG TYPES
 ─────────────────────────────────────────────────────────────────────────────

  /input/* and /cmd/*                      VehicleState (polaris_msgs)
  (individual std_msgs topics)             ┌─────────────────────────┐
  ┌─────────────────────────┐              │ steering_angle: f32     │
  │ /*/steering:  Float32   │              │ throttle_position: f32  │
  │ /*/throttle:  Float32   │              │ brake_pressure: f32     │
  │ /*/brake:     Float32   │              │ gear_current: u8        │
  │ /*/gear:      UInt8     │              │ handbrake_engaged: bool │
  │ /*/handbrake: Bool      │              │ speed_mps: f32          │
  └─────────────────────────┘              │ heading_deg: f32        │
                                           │ lat: f64, lon: f64      │
  /vehicle/imu: sensor_msgs/Imu           │ mode: u8                │
  /vehicle/gps: sensor_msgs/NavSatFix     └─────────────────────────┘


  WIRE PROTOCOLS
 ─────────────────────────────────────────────────────────────────────────────

  Wheel ↔ Bridge (QUIC:9876, little-endian)

  UP (14 bytes — Mac → fastgpu):
  ┌───────┬───────┬───────┬────┬────┐
  │ f32   │ f32   │ f32   │ u8 │ u8 │
  │ steer │ throt │ brake │gear│hbrk│
  └───────┴───────┴───────┴────┴────┘

  DOWN (72 bytes — fastgpu → Mac):
  ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬────┬────┬────┬────┐
  │ f32   │ f32   │ f64   │ f64   │ f32   │ f32   │ f32   │ u8 │ u8 │ u8 │ u8 │
  │ speed │ head  │ lat   │ lon   │ s_str │ s_thr │ s_brk │s_gr│s_hb│gear│mode│
  └───────┴───────┴───────┴───────┴───────┴───────┴───────┴────┴────┴────┴────┘
  ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┐
  │ f64   │ f32   │ f32   │ f32   │ f32   │ f32   │ f32   │
  │ alt   │ acc_x │ acc_y │ acc_z │gyr_x │gyr_y │gyr_z │
  └───────┴───────┴───────┴───────┴───────┴───────┴───────┘


  PWM CHANNEL MAPPING (pixhawk.rs)
 ─────────────────────────────────────────────────────────────────────────────

  Channel  Actuator    PWM Range         Mapping
  ───────  ────────    ─────────         ───────
  Ch1      Steering    1000-2000         centered at 1500, -1..1 → 1000..2000
  Ch2      Brake       1000-2000         0..1 → 1000..2000
  Ch3      Throttle    1500-2000         0..1 → 1500..2000
  Ch4      Gear        1000/1250/1500/1750  P=1000 R=1250 N=1500 D=1750
  Ch5      Handbrake   1000 or 2000      released=1000, engaged=2000
```
