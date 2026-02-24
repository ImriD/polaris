mod gamepad;

use crossterm::event::{self, Event, KeyCode, KeyEvent};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::ExecutableCommand;
use futures::StreamExt;
use r2r::QosProfile;
use ratatui::prelude::*;
use ratatui::widgets::*;
use std::io::stdout;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

const GEAR_NAMES: [&str; 4] = ["P", "R", "N", "D"];

/// Shared atomic counters for topic message rates — one per topic
struct TopicRates {
  input_steering: AtomicU64,
  input_throttle: AtomicU64,
  input_brake: AtomicU64,
  input_gear: AtomicU64,
  input_handbrake: AtomicU64,
  cmd_steering: AtomicU64,
  cmd_throttle: AtomicU64,
  cmd_brake: AtomicU64,
  cmd_gear: AtomicU64,
  cmd_handbrake: AtomicU64,
  vehicle_state: AtomicU64,
  vehicle_mode: AtomicU64,
  vehicle_imu: AtomicU64,
  vehicle_gps: AtomicU64,
}

impl TopicRates {
  fn new() -> Self {
    Self {
      input_steering: AtomicU64::new(0), input_throttle: AtomicU64::new(0),
      input_brake: AtomicU64::new(0), input_gear: AtomicU64::new(0),
      input_handbrake: AtomicU64::new(0), cmd_steering: AtomicU64::new(0),
      cmd_throttle: AtomicU64::new(0), cmd_brake: AtomicU64::new(0),
      cmd_gear: AtomicU64::new(0), cmd_handbrake: AtomicU64::new(0),
      vehicle_state: AtomicU64::new(0), vehicle_mode: AtomicU64::new(0),
      vehicle_imu: AtomicU64::new(0), vehicle_gps: AtomicU64::new(0),
    }
  }
}

/// Snapshot of Hz values computed from counters
#[derive(Default, Clone)]
struct RateSnapshot {
  input_hz: f32,   // avg across 5 /input/* topics
  cmd_hz: f32,     // avg across 5 /cmd/* topics
  state_hz: f32,   // /vehicle/state
  mode_hz: f32,    // /vehicle/mode
  imu_hz: f32,     // /vehicle/imu
  gps_hz: f32,     // /vehicle/gps
}
const STEER_INCREMENT: f32 = 0.05;
const STEER_DECAY: f32 = 0.8;
const THROTTLE_INCREMENT: f32 = 0.1;
const BRAKE_INCREMENT: f32 = 0.2;
const MAX_STEER: f32 = 7.854; // 450 degrees in radians

struct DashboardState {
  speed: f32,
  heading: f32,
  lat: f64,
  lon: f64,
  mode_str: String,
  gear_display: String,
  last_state_time: Instant,
  connected: bool,
  steering: f32,
  throttle: f32,
  brake: f32,
  gear: u8,
  handbrake: bool,
  quit: bool,
  wheel_connected: bool,
  remote_input: bool, // true when /input/* topics are being received (e.g. from wheel_bridge_node)
  // Actuator state (from /cmd/* — what reaches the servos after safety filter)
  safe_steering: f32,
  safe_throttle: f32,
  safe_brake: f32,
  safe_gear: u8,
  safe_handbrake: bool,
  rates: RateSnapshot,
  altitude: f64,
  acc: [f32; 3],
  gyro: [f32; 3],
  pixhawk_connected: bool,
}

impl DashboardState {
  fn new() -> Self {
    Self {
      speed: 0.0, heading: 0.0, lat: 0.0, lon: 0.0,
      mode_str: "UNKNOWN".into(), gear_display: "P".into(),
      last_state_time: Instant::now(), connected: false,
      steering: 0.0, throttle: 0.0, brake: 0.0,
      gear: 3, handbrake: false,
      quit: false, wheel_connected: false, remote_input: false,
      safe_steering: 0.0, safe_throttle: 0.0, safe_brake: 0.0,
      safe_gear: 3, safe_handbrake: false, rates: RateSnapshot::default(),
      altitude: 0.0, acc: [0.0; 3], gyro: [0.0; 3], pixhawk_connected: false,
    }
  }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
  env_logger::init();
  let monitor_mode = std::env::args().any(|a| a == "--monitor");
  let shared = Arc::new(Mutex::new(DashboardState::new()));
  // Spawn tokio+r2r in background thread
  let st = shared.clone();
  std::thread::spawn(move || {
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async move {
      if let Err(e) = ros_spin(st, monitor_mode).await {
        log::error!("ROS2 thread error: {}", e);
      }
    });
  });

  // Main thread: crossterm/ratatui render loop
  let mut gilrs = if !monitor_mode { gilrs::Gilrs::new().ok() } else { None };
  let mut gamepad = gamepad::GamepadState::new(MAX_STEER, 0.05);

  enable_raw_mode()?;
  stdout().execute(EnterAlternateScreen)?;
  let mut terminal = Terminal::new(CrosstermBackend::new(stdout()))?;

  loop {
    {
      let s = shared.lock().unwrap();
      if s.quit { break; }
    }

    // 1. Poll gamepad (always, non-blocking)
    if let Some(ref mut g) = gilrs {
      gamepad.poll(g);
      let has_gamepad = g.gamepads().next().is_some();
      let mut s = shared.lock().unwrap();
      if has_gamepad {
        s.steering = gamepad.steering;
        s.throttle = gamepad.throttle;
        s.brake = gamepad.brake;
        s.gear = gamepad.gear;
        s.handbrake = gamepad.handbrake;
        s.wheel_connected = true;
      } else {
        s.wheel_connected = false;
      }
    }

    // 2. Keyboard input + decay (only when no wheel)
    let wheel = shared.lock().unwrap().wheel_connected;
    if event::poll(Duration::from_millis(33))? {
      if let Event::Key(KeyEvent { code, .. }) = event::read()? {
        let mut s = shared.lock().unwrap();
        match code {
          // Navigation keys always work
          KeyCode::Char('q') | KeyCode::Char('Q') => { s.quit = true; }
          // Control keys only when no wheel
          _ if !wheel => match code {
            KeyCode::Char('w') | KeyCode::Char('W') => { s.throttle = (s.throttle + THROTTLE_INCREMENT).min(1.0); s.brake = 0.0; }
            KeyCode::Char('s') | KeyCode::Char('S') => { s.brake = (s.brake + BRAKE_INCREMENT).min(1.0); s.throttle = 0.0; }
            KeyCode::Char('a') | KeyCode::Char('A') => { s.steering += STEER_INCREMENT; }
            KeyCode::Char('d') | KeyCode::Char('D') => { s.steering -= STEER_INCREMENT; }
            KeyCode::Char(' ') => { s.throttle = 0.0; s.brake = 1.0; s.steering = 0.0; }
            KeyCode::Char('g') | KeyCode::Char('G') => { s.gear = (s.gear + 1) % 4; }
            KeyCode::Char('h') | KeyCode::Char('H') => { s.handbrake = !s.handbrake; }
            _ => {}
          }
          _ => {}
        }
      }
    }

    // Decay controls — only for keyboard (wheel and remote provide absolute values)
    {
      let mut s = shared.lock().unwrap();
      if !s.wheel_connected && !s.remote_input {
        s.steering *= STEER_DECAY;
        s.throttle = (s.throttle - 0.05).max(0.0);
        s.brake = (s.brake - 0.05).max(0.0);
      }
      s.connected = s.last_state_time.elapsed().as_millis() < 2000;
    }

    let s = shared.lock().unwrap();
    terminal.draw(|f| draw(f, &s))?;
  }

  disable_raw_mode()?;
  stdout().execute(LeaveAlternateScreen)?;
  Ok(())
}

fn draw(f: &mut Frame, s: &DashboardState) {
  let area = f.area();
  let chunks = Layout::default().direction(Direction::Vertical).constraints([
    Constraint::Length(1), // header
    Constraint::Length(4), // connections
    Constraint::Length(7), // actuators
    Constraint::Length(6), // telemetry
    Constraint::Length(1), // footer
    Constraint::Min(0),    // absorb
  ]).split(area);

  // Header
  let input_src = if s.wheel_connected { "WHEEL" } else if s.remote_input { "REMOTE" } else { "KEYBOARD" };
  let right = format!("{} ", input_src);
  let pad = (area.width as usize).saturating_sub(17 + right.len());
  f.render_widget(
    Paragraph::new(Line::from(vec![
      Span::styled(" POLARIS REMOTE", Style::default().bold()),
      Span::raw(format!("{:pad$}{}", "", right)),
    ])),
    chunks[0],
  );

  // Connections
  let ok = Style::default().fg(Color::Green);
  let dead = Style::default().fg(Color::Red);
  let r = &s.rates;
  let dot = |hz: f32| if hz > 0.5 { Span::styled("●", ok) } else { Span::styled("●", dead) };
  let hz_fmt = |hz: f32| if hz > 0.5 { format!(" {:.0}Hz", hz) } else { " --".into() };
  f.render_widget(
    Paragraph::new(vec![
      Line::from(vec![
        Span::raw("  WHEEL "), dot(r.input_hz), Span::raw(hz_fmt(r.input_hz)),
        Span::raw("   CONTROL "), dot(r.cmd_hz), Span::raw(hz_fmt(r.cmd_hz)),
        Span::raw("   PIXHAWK "), dot(r.state_hz), Span::raw(hz_fmt(r.state_hz)),
      ]),
      Line::from(vec![
        Span::raw("  GPS "), dot(r.gps_hz), Span::raw(hz_fmt(r.gps_hz)),
        Span::raw("     IMU "), dot(r.imu_hz), Span::raw(hz_fmt(r.imu_hz)),
        Span::raw(format!("       MODE: {}", s.mode_str)),
      ]),
    ]).block(Block::bordered().title(" CONNECTIONS ")),
    chunks[1],
  );

  // Actuators
  let sp = s.steering / MAX_STEER;
  let bar_w = 20usize;
  let center = bar_w / 2;
  let pos = ((center as f32 + sp.clamp(-1.0, 1.0) * center as f32) as usize).clamp(0, bar_w - 1);
  let steer_bar: String = (0..bar_w).map(|i| if i == pos { '█' } else { '─' }).collect();
  let thr_n = (s.throttle.clamp(0.0, 1.0) * 20.0) as usize;
  let brk_n = (s.brake.clamp(0.0, 1.0) * 20.0) as usize;
  let thr_bar = format!("{}{}", "█".repeat(thr_n), "░".repeat(20 - thr_n));
  let brk_bar = format!("{}{}", "█".repeat(brk_n), "░".repeat(20 - brk_n));
  let gear_disp: String = GEAR_NAMES.iter().enumerate()
    .map(|(i, &g)| if i == s.gear as usize { format!(">{}< ", g) } else { format!(" {}  ", g) })
    .collect();
  let steer_pwm = (1500.0 + sp * 500.0) as u16;
  let gear_pwm: [u16; 4] = [1000, 1250, 1500, 1750];
  f.render_widget(
    Paragraph::new(vec![
      Line::from(format!("  CH1 STEERING   ◀├{}┤▶ {:+5.1}°   PWM {}", steer_bar, s.steering.to_degrees(), steer_pwm)),
      Line::from(format!("  CH2 THROTTLE    ├{}┤  {:3.0}%     PWM {}", thr_bar, s.throttle * 100.0, (1000.0 + s.throttle * 1000.0) as u16)),
      Line::from(format!("  CH3 BRAKE       ├{}┤  {:3.0}%     PWM {}", brk_bar, s.brake * 100.0, (1000.0 + s.brake * 1000.0) as u16)),
      Line::from(format!("  CH4 GEAR        {}           PWM {}", gear_disp, gear_pwm[s.gear.min(3) as usize])),
      Line::from(format!("  CH5 HANDBRAKE   {} {}                    PWM {}",
        if s.handbrake { "●" } else { "○" }, if s.handbrake { "ENGAGED " } else { "RELEASED" },
        if s.handbrake { 2000u16 } else { 1000 })),
    ]).block(Block::bordered().title(" ACTUATORS ")),
    chunks[2],
  );

  // Telemetry
  f.render_widget(
    Paragraph::new(vec![
      Line::from(format!("  Speed  {:5.1} km/h  ({:.2} m/s)     Heading  {:5.1}°", s.speed * 3.6, s.speed, s.heading)),
      Line::from(format!("  GPS    {:.6}, {:.6}       Alt  {:.1} m", s.lat, s.lon, s.altitude)),
      Line::from(format!("  IMU acc   x {:+6.2}   y {:+6.2}   z {:+6.2}  m/s²", s.acc[0], s.acc[1], s.acc[2])),
      Line::from(format!("  IMU gyro  x {:+6.3}   y {:+6.3}   z {:+6.3}  rad/s", s.gyro[0], s.gyro[1], s.gyro[2])),
    ]).block(Block::bordered().title(" TELEMETRY ")),
    chunks[3],
  );

  // Footer
  f.render_widget(Paragraph::new(" G=gear  H=handbrake  Space=stop  Q=quit"), chunks[4]);
}

async fn ros_spin(shared: Arc<Mutex<DashboardState>>, monitor_mode: bool) -> Result<(), Box<dyn std::error::Error>> {
  let ctx = r2r::Context::create()?;
  let node_name = if monitor_mode { "tui_monitor" } else { "tui_dashboard" };
  let mut node = r2r::Node::create(ctx, node_name, "")?;

  let reliable = QosProfile::default();
  let rates = Arc::new(TopicRates::new());

  let state_sub = node.subscribe::<r2r::polaris_msgs::msg::VehicleState>("/vehicle/state", QosProfile::sensor_data())?;
  let mode_sub = node.subscribe::<r2r::std_msgs::msg::String>("/vehicle/mode", reliable.clone())?;

  // Subscribe to /cmd/* (filtered — what reaches servos)
  let cmd_steer_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/steering", reliable.clone())?;
  let cmd_throt_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/throttle", reliable.clone())?;
  let cmd_brake_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/brake", reliable.clone())?;
  let cmd_gear_sub = node.subscribe::<r2r::std_msgs::msg::UInt8>("/cmd/gear", reliable.clone())?;
  let cmd_hbrake_sub = node.subscribe::<r2r::std_msgs::msg::Bool>("/cmd/handbrake", reliable.clone())?;

  // Subscribe to /input/* so monitor can display operator commands
  let in_steer_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/input/steering", reliable.clone())?;
  let in_throt_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/input/throttle", reliable.clone())?;
  let in_brake_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/input/brake", reliable.clone())?;
  let in_gear_sub = node.subscribe::<r2r::std_msgs::msg::UInt8>("/input/gear", reliable.clone())?;
  let in_hbrake_sub = node.subscribe::<r2r::std_msgs::msg::Bool>("/input/handbrake", reliable.clone())?;

  // Handle /cmd/* (what reaches the actuators)
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    cmd_steer_sub.for_each(|msg| { r.cmd_steering.fetch_add(1, Ordering::Relaxed); st.lock().unwrap().safe_steering = msg.data; std::future::ready(()) }).await;
  });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    cmd_throt_sub.for_each(|msg| { r.cmd_throttle.fetch_add(1, Ordering::Relaxed); st.lock().unwrap().safe_throttle = msg.data; std::future::ready(()) }).await;
  });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    cmd_brake_sub.for_each(|msg| { r.cmd_brake.fetch_add(1, Ordering::Relaxed); st.lock().unwrap().safe_brake = msg.data; std::future::ready(()) }).await;
  });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    cmd_gear_sub.for_each(|msg| { r.cmd_gear.fetch_add(1, Ordering::Relaxed); st.lock().unwrap().safe_gear = msg.data; std::future::ready(()) }).await;
  });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    cmd_hbrake_sub.for_each(|msg| { r.cmd_handbrake.fetch_add(1, Ordering::Relaxed); st.lock().unwrap().safe_handbrake = msg.data; std::future::ready(()) }).await;
  });

  // Handle /vehicle/state
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    state_sub.for_each(|msg| {
      r.vehicle_state.fetch_add(1, Ordering::Relaxed);
      let mut s = st.lock().unwrap();
      s.speed = msg.speed_mps;
      s.heading = msg.heading_deg;
      s.lat = msg.lat;
      s.lon = msg.lon;
      s.gear_display = if (msg.gear_current as usize) < GEAR_NAMES.len() {
        GEAR_NAMES[msg.gear_current as usize].to_string()
      } else { "?".to_string() };
      s.last_state_time = Instant::now();
      s.connected = true;
      std::future::ready(())
    }).await;
  });

  // Handle /vehicle/mode
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    mode_sub.for_each(|msg| {
      r.vehicle_mode.fetch_add(1, Ordering::Relaxed);
      st.lock().unwrap().mode_str = msg.data;
      std::future::ready(())
    }).await;
  });

  // Handle /vehicle/imu (sensor_msgs/Imu)
  let imu_sub = node.subscribe::<r2r::sensor_msgs::msg::Imu>("/vehicle/imu", QosProfile::sensor_data())?;
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    imu_sub.for_each(|msg| {
      r.vehicle_imu.fetch_add(1, Ordering::Relaxed);
      let mut s = st.lock().unwrap();
      s.acc = [msg.linear_acceleration.x as f32, msg.linear_acceleration.y as f32, msg.linear_acceleration.z as f32];
      s.gyro = [msg.angular_velocity.x as f32, msg.angular_velocity.y as f32, msg.angular_velocity.z as f32];
      s.pixhawk_connected = true;
      std::future::ready(())
    }).await;
  });

  // Handle /vehicle/gps (sensor_msgs/NavSatFix)
  let gps_sub = node.subscribe::<r2r::sensor_msgs::msg::NavSatFix>("/vehicle/gps", QosProfile::sensor_data())?;
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    gps_sub.for_each(|msg| {
      r.vehicle_gps.fetch_add(1, Ordering::Relaxed);
      st.lock().unwrap().altitude = msg.altitude;
      std::future::ready(())
    }).await;
  });

  // Always read /input/* — shows what the active input source is sending
  // When local gamepad is connected, gilrs overwrites these at 60Hz (takes priority)
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move { in_steer_sub.for_each(|msg| { r.input_steering.fetch_add(1, Ordering::Relaxed); let mut s = st.lock().unwrap(); if !s.wheel_connected { s.steering = msg.data; s.remote_input = true; } std::future::ready(()) }).await; });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move { in_throt_sub.for_each(|msg| { r.input_throttle.fetch_add(1, Ordering::Relaxed); let mut s = st.lock().unwrap(); if !s.wheel_connected { s.throttle = msg.data; } std::future::ready(()) }).await; });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move { in_brake_sub.for_each(|msg| { r.input_brake.fetch_add(1, Ordering::Relaxed); let mut s = st.lock().unwrap(); if !s.wheel_connected { s.brake = msg.data; } std::future::ready(()) }).await; });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move { in_gear_sub.for_each(|msg| { r.input_gear.fetch_add(1, Ordering::Relaxed); let mut s = st.lock().unwrap(); if !s.wheel_connected { s.gear = msg.data; } std::future::ready(()) }).await; });
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move { in_hbrake_sub.for_each(|msg| { r.input_handbrake.fetch_add(1, Ordering::Relaxed); let mut s = st.lock().unwrap(); if !s.wheel_connected { s.handbrake = msg.data; } std::future::ready(()) }).await; });

  // 1Hz rate sampler — swap-and-reset counters, compute Hz averages
  let st = shared.clone(); let r = rates.clone();
  tokio::spawn(async move {
    loop {
      tokio::time::sleep(Duration::from_secs(1)).await;
      let input_total = r.input_steering.swap(0, Ordering::Relaxed)
        + r.input_throttle.swap(0, Ordering::Relaxed)
        + r.input_brake.swap(0, Ordering::Relaxed)
        + r.input_gear.swap(0, Ordering::Relaxed)
        + r.input_handbrake.swap(0, Ordering::Relaxed);
      let cmd_total = r.cmd_steering.swap(0, Ordering::Relaxed)
        + r.cmd_throttle.swap(0, Ordering::Relaxed)
        + r.cmd_brake.swap(0, Ordering::Relaxed)
        + r.cmd_gear.swap(0, Ordering::Relaxed)
        + r.cmd_handbrake.swap(0, Ordering::Relaxed);
      let state_count = r.vehicle_state.swap(0, Ordering::Relaxed);
      let mode_count = r.vehicle_mode.swap(0, Ordering::Relaxed);
      let imu_count = r.vehicle_imu.swap(0, Ordering::Relaxed);
      let gps_count = r.vehicle_gps.swap(0, Ordering::Relaxed);
      let mut s = st.lock().unwrap();
      s.rates.input_hz = input_total as f32 / 5.0;
      s.rates.cmd_hz = cmd_total as f32 / 5.0;
      s.rates.state_hz = state_count as f32;
      s.rates.mode_hz = mode_count as f32;
      s.rates.imu_hz = imu_count as f32;
      s.rates.gps_hz = gps_count as f32;
      if imu_count == 0 && gps_count == 0 { s.pixhawk_connected = false; }
    }
  });

  if !monitor_mode {
    // Publish to /input/* only when local gamepad is connected
    let in_steer_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/steering", reliable.clone())?;
    let in_throt_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/throttle", reliable.clone())?;
    let in_brake_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/brake", reliable.clone())?;
    let in_gear_pub = node.create_publisher::<r2r::std_msgs::msg::UInt8>("/input/gear", reliable.clone())?;
    let in_hbrake_pub = node.create_publisher::<r2r::std_msgs::msg::Bool>("/input/handbrake", reliable)?;
    let mut publish_timer = node.create_wall_timer(Duration::from_millis(33))?;
    let st = shared.clone();
    tokio::spawn(async move {
      loop {
        let _ = publish_timer.tick().await;
        let s = st.lock().unwrap();
        if !s.wheel_connected { continue; } // no local input — don't fight remote sources
        let _ = in_steer_pub.publish(&r2r::std_msgs::msg::Float32 { data: s.steering.clamp(-MAX_STEER, MAX_STEER) });
        let _ = in_throt_pub.publish(&r2r::std_msgs::msg::Float32 { data: s.throttle });
        let _ = in_brake_pub.publish(&r2r::std_msgs::msg::Float32 { data: s.brake });
        let _ = in_gear_pub.publish(&r2r::std_msgs::msg::UInt8 { data: s.gear });
        let _ = in_hbrake_pub.publish(&r2r::std_msgs::msg::Bool { data: s.handbrake });
      }
    });
  }

  loop {
    node.spin_once(Duration::from_millis(10));
    tokio::time::sleep(Duration::from_millis(1)).await;
  }
}
