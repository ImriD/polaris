use crossterm::event::{self, Event, KeyCode, KeyEvent};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::ExecutableCommand;
use ratatui::prelude::*;
use ratatui::widgets::*;
use quinn::rustls;
use rustls::pki_types::{CertificateDer, ServerName, UnixTime};
use std::io::stdout;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const GEAR_NAMES: [&str; 4] = ["P", "R", "N", "D"];
const MAX_STEER: f32 = 7.854; // 450 degrees in radians
const PACKET_HZ: u64 = 50;
const DEADZONE: f32 = 0.02;

// G923 SDL2 axis indices
const AXIS_STEERING: u32 = 0;
const AXIS_THROTTLE: u32 = 2;
const AXIS_BRAKE: u32 = 1;

// G923 SDL2 button indices (PS version)
const BTN_RIGHT_PADDLE: u32 = 5; // R1
const BTN_LEFT_PADDLE: u32 = 4;  // L1
const BTN_CIRCLE: u32 = 1;
const BTN_OPTIONS: u32 = 9;

/// 14-byte packet
struct WheelPacket {
  steering: f32,
  throttle: f32,
  brake: f32,
  gear: u8,
  handbrake: u8,
}

impl WheelPacket {
  fn to_bytes(&self) -> [u8; 14] {
    let mut buf = [0u8; 14];
    buf[0..4].copy_from_slice(&self.steering.to_le_bytes());
    buf[4..8].copy_from_slice(&self.throttle.to_le_bytes());
    buf[8..12].copy_from_slice(&self.brake.to_le_bytes());
    buf[12] = self.gear;
    buf[13] = self.handbrake;
    buf
  }
}

struct VehicleDown {
  speed_mps: f32,
  heading_deg: f32,
  lat: f64,
  lon: f64,
  safe_steering: f32,
  safe_throttle: f32,
  safe_brake: f32,
  safe_gear: u8,
  safe_handbrake: u8,
  gear_current: u8,
  mode: u8,
  altitude: f64,
  acc_x: f32, acc_y: f32, acc_z: f32,
  gyro_x: f32, gyro_y: f32, gyro_z: f32,
}

impl VehicleDown {
  fn parse(buf: &[u8; 72]) -> Self {
    Self {
      speed_mps: f32::from_le_bytes(buf[0..4].try_into().unwrap()),
      heading_deg: f32::from_le_bytes(buf[4..8].try_into().unwrap()),
      lat: f64::from_le_bytes(buf[8..16].try_into().unwrap()),
      lon: f64::from_le_bytes(buf[16..24].try_into().unwrap()),
      safe_steering: f32::from_le_bytes(buf[24..28].try_into().unwrap()),
      safe_throttle: f32::from_le_bytes(buf[28..32].try_into().unwrap()),
      safe_brake: f32::from_le_bytes(buf[32..36].try_into().unwrap()),
      safe_gear: buf[36],
      safe_handbrake: buf[37],
      gear_current: buf[38],
      mode: buf[39],
      altitude: f64::from_le_bytes(buf[40..48].try_into().unwrap()),
      acc_x: f32::from_le_bytes(buf[48..52].try_into().unwrap()),
      acc_y: f32::from_le_bytes(buf[52..56].try_into().unwrap()),
      acc_z: f32::from_le_bytes(buf[56..60].try_into().unwrap()),
      gyro_x: f32::from_le_bytes(buf[60..64].try_into().unwrap()),
      gyro_y: f32::from_le_bytes(buf[64..68].try_into().unwrap()),
      gyro_z: f32::from_le_bytes(buf[68..72].try_into().unwrap()),
    }
  }
}

struct State {
  steering: f32,
  throttle: f32,
  brake: f32,
  gear: u8,
  handbrake: bool,
  gamepad_name: String,
  packets_sent: u64,
  quit: bool,
  raw_axes: [i16; 4],
  connected: bool,
  speed_mps: f32,
  heading_deg: f32,
  mode: u8,
  lat: f64,
  lon: f64,
  altitude: f64,
  acc: [f32; 3],
  gyro: [f32; 3],
  safe_steering: f32,
  safe_throttle: f32,
  safe_brake: f32,
  safe_gear: u8,
  safe_handbrake: u8,
  gear_current: u8,
}

impl State {
  fn new() -> Self {
    Self {
      steering: 0.0, throttle: 0.0, brake: 0.0, gear: 3, handbrake: false,
      gamepad_name: "none".into(), packets_sent: 0, quit: false, raw_axes: [0; 4],
      connected: false, speed_mps: 0.0, heading_deg: 0.0, mode: 0,
      lat: 0.0, lon: 0.0, altitude: 0.0, acc: [0.0; 3], gyro: [0.0; 3],
      safe_steering: 0.0, safe_throttle: 0.0, safe_brake: 0.0,
      safe_gear: 3, safe_handbrake: 0, gear_current: 3,
    }
  }

  fn packet(&self) -> WheelPacket {
    WheelPacket {
      steering: self.steering, throttle: self.throttle, brake: self.brake,
      gear: self.gear, handbrake: self.handbrake as u8,
    }
  }
}

/// Normalize SDL2 axis (-32768..32767) to 0..1 for pedals.
/// Pedal at rest = 32767 (released), fully pressed = -32768.
fn pedal_to_normalized(raw: i16) -> f32 {
  (1.0 - (raw as f32 + 32768.0) / 65535.0).clamp(0.0, 1.0)
}

/// Normalize SDL2 axis to -1..1 for steering, with deadzone.
fn steering_to_normalized(raw: i16) -> f32 {
  let v = raw as f32 / 32767.0;
  if v.abs() < DEADZONE { 0.0 } else { v.signum() * (v.abs() - DEADZONE) / (1.0 - DEADZONE) }
}

#[derive(Debug)]
struct SkipServerVerification(Arc<rustls::crypto::CryptoProvider>);

impl SkipServerVerification {
  fn new() -> Arc<Self> {
    Arc::new(Self(Arc::new(rustls::crypto::ring::default_provider())))
  }
}

impl rustls::client::danger::ServerCertVerifier for SkipServerVerification {
  fn verify_server_cert(&self, _end_entity: &CertificateDer<'_>, _intermediates: &[CertificateDer<'_>],
    _server_name: &ServerName<'_>, _ocsp: &[u8], _now: UnixTime,
  ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
    Ok(rustls::client::danger::ServerCertVerified::assertion())
  }
  fn verify_tls12_signature(&self, message: &[u8], cert: &CertificateDer<'_>,
    dss: &rustls::DigitallySignedStruct,
  ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
    rustls::crypto::verify_tls12_signature(message, cert, dss, &self.0.signature_verification_algorithms)
  }
  fn verify_tls13_signature(&self, message: &[u8], cert: &CertificateDer<'_>,
    dss: &rustls::DigitallySignedStruct,
  ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
    rustls::crypto::verify_tls13_signature(message, cert, dss, &self.0.signature_verification_algorithms)
  }
  fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
    self.0.signature_verification_algorithms.supported_schemes()
  }
}

fn insecure_client_config() -> quinn::ClientConfig {
  let crypto = rustls::ClientConfig::builder()
    .dangerous()
    .with_custom_certificate_verifier(SkipServerVerification::new())
    .with_no_client_auth();
  quinn::ClientConfig::new(Arc::new(quinn::crypto::rustls::QuicClientConfig::try_from(crypto).unwrap()))
}

async fn quic_loop(target: String, state: Arc<Mutex<State>>) {
  let mut endpoint = quinn::Endpoint::client("0.0.0.0:0".parse().unwrap()).unwrap();
  endpoint.set_default_client_config(insecure_client_config());
  let interval = Duration::from_millis(1000 / PACKET_HZ);

  loop {
    let addr: std::net::SocketAddr = match target.parse() {
      Ok(a) => a,
      Err(_) => match std::net::ToSocketAddrs::to_socket_addrs(&target.as_str()) {
        Ok(mut addrs) => match addrs.find(|a| a.is_ipv4()) {
          Some(a) => a,
          None => { tokio::time::sleep(Duration::from_secs(1)).await; continue; }
        },
        Err(_) => { tokio::time::sleep(Duration::from_secs(1)).await; continue; }
      },
    };

    match endpoint.connect(addr, "fastgpu.local") {
      Ok(connecting) => match connecting.await {
        Ok(conn) => {
          log::info!("QUIC connected to {}", conn.remote_address());
          state.lock().unwrap().connected = true;
          match conn.open_bi().await {
            Ok((mut send, mut recv)) => {
              loop {
                let pkt = state.lock().unwrap().packet();
                // Client MUST write before server can accept_bi
                if send.write_all(&pkt.to_bytes()).await.is_err() { break; }
                state.lock().unwrap().packets_sent += 1;
                let mut buf = [0u8; 72];
                if recv.read_exact(&mut buf).await.is_err() { break; }
                let v = VehicleDown::parse(&buf);
                {
                  let mut s = state.lock().unwrap();
                  s.speed_mps = v.speed_mps; s.heading_deg = v.heading_deg;
                  s.mode = v.mode; s.lat = v.lat; s.lon = v.lon; s.altitude = v.altitude;
                  s.acc = [v.acc_x, v.acc_y, v.acc_z];
                  s.gyro = [v.gyro_x, v.gyro_y, v.gyro_z];
                  s.safe_steering = v.safe_steering; s.safe_throttle = v.safe_throttle;
                  s.safe_brake = v.safe_brake; s.safe_gear = v.safe_gear;
                  s.safe_handbrake = v.safe_handbrake; s.gear_current = v.gear_current;
                } // lock released here, before await
                tokio::time::sleep(interval).await;
              }
            }
            Err(e) => log::warn!("open_bi failed: {e}"),
          }
        }
        Err(e) => log::warn!("QUIC connect failed: {e}"),
      },
      Err(e) => log::warn!("QUIC connect config error: {e}"),
    }
    state.lock().unwrap().connected = false;
    log::info!("QUIC disconnected, retrying in 1s...");
    tokio::time::sleep(Duration::from_secs(1)).await;
  }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  env_logger::init();

  let target = std::env::args().nth(1).unwrap_or_else(|| "fastgpu.local:9876".into());

  // SDL2 init
  let sdl = sdl2::init().map_err(|e| format!("SDL2: {e}"))?;
  let js_sys = sdl.joystick().map_err(|e| format!("joystick: {e}"))?;

  let n = js_sys.num_joysticks().map_err(|e| format!("enum: {e}"))?;
  if n == 0 { return Err("No joystick found. Is the G923 connected?".into()); }

  let js = js_sys.open(0).map_err(|e| format!("open: {e}"))?;
  eprintln!("Opened: {} ({} axes, {} buttons)", js.name(), js.num_axes(), js.num_buttons());

  let mut pump = sdl.event_pump().map_err(|e| format!("event pump: {e}"))?;
  let state = Arc::new(Mutex::new(State::new()));
  state.lock().unwrap().gamepad_name = js.name();

  // QUIC task — runs concurrently, auto-reconnects
  let st = state.clone();
  let tgt = target.clone();
  tokio::spawn(quic_loop(tgt, st));

  // TUI — SDL2 must stay on main thread (macOS requirement)
  enable_raw_mode()?;
  stdout().execute(EnterAlternateScreen)?;
  let mut terminal = Terminal::new(CrosstermBackend::new(stdout()))?;
  let mut prev_buttons = [false; 25];

  tokio::task::block_in_place(|| -> Result<(), Box<dyn std::error::Error>> {
    loop {
      if state.lock().unwrap().quit { break; }

      // Pump SDL2 events (required on macOS main thread)
      pump.pump_events();

      // Read axes
      {
        let mut s = state.lock().unwrap();
        for i in 0..js.num_axes().min(4) {
          s.raw_axes[i as usize] = js.axis(i).unwrap_or(0);
        }
        s.steering = steering_to_normalized(s.raw_axes[AXIS_STEERING as usize]) * MAX_STEER;
        s.throttle = pedal_to_normalized(s.raw_axes[AXIS_THROTTLE as usize]);
        s.brake = pedal_to_normalized(s.raw_axes[AXIS_BRAKE as usize]);
      }

      // Read buttons (edge-triggered for gear/handbrake)
      {
        let mut s = state.lock().unwrap();
        let mut cur = [false; 25];
        for i in 0..js.num_buttons().min(25) {
          cur[i as usize] = js.button(i).unwrap_or(false);
        }
        // Right paddle → gear up
        if cur[BTN_RIGHT_PADDLE as usize] && !prev_buttons[BTN_RIGHT_PADDLE as usize] {
          s.gear = (s.gear + 1).min(3);
        }
        // Left paddle → gear down
        if cur[BTN_LEFT_PADDLE as usize] && !prev_buttons[BTN_LEFT_PADDLE as usize] {
          s.gear = s.gear.saturating_sub(1);
        }
        // Circle → handbrake toggle
        if cur[BTN_CIRCLE as usize] && !prev_buttons[BTN_CIRCLE as usize] {
          s.handbrake = !s.handbrake;
        }
        // Options → e-stop
        if cur[BTN_OPTIONS as usize] && !prev_buttons[BTN_OPTIONS as usize] {
          s.throttle = 0.0;
          s.brake = 1.0;
        }
        prev_buttons = cur;
      }

      // Poll keyboard
      if event::poll(Duration::from_millis(16))? {
        if let Event::Key(KeyEvent { code, .. }) = event::read()? {
          let mut s = state.lock().unwrap();
          match code {
            KeyCode::Char('q' | 'Q') => s.quit = true,
            KeyCode::Char('g' | 'G') => s.gear = (s.gear + 1) % 4,
            KeyCode::Char('h' | 'H') => s.handbrake = !s.handbrake,
            _ => {}
          }
        }
      }

      let s = state.lock().unwrap();
      terminal.draw(|f| draw(f, &s, &target))?;
    }
    Ok(())
  })?;

  disable_raw_mode()?;
  stdout().execute(LeaveAlternateScreen)?;
  Ok(())
}

fn draw(f: &mut Frame, s: &State, target: &str) {
  let area = f.area();
  let chunks = Layout::default().direction(Direction::Vertical).constraints([
    Constraint::Length(1), // header
    Constraint::Length(4), // connections
    Constraint::Length(7), // actuators
    Constraint::Length(6), // telemetry
    Constraint::Length(2), // footer
    Constraint::Min(0),    // absorb
  ]).split(area);

  // Header
  let right = format!("{} · {} ", s.gamepad_name, target);
  let pad = (area.width as usize).saturating_sub(16 + right.len());
  f.render_widget(
    Paragraph::new(Line::from(vec![
      Span::styled(" POLARIS WHEEL", Style::default().bold()),
      Span::raw(format!("{:pad$}{}", "", right)),
    ])),
    chunks[0],
  );

  // Connections
  let ok = Style::default().fg(Color::Green);
  let dead = Style::default().fg(Color::Red);
  f.render_widget(
    Paragraph::new(vec![
      Line::from(vec![
        Span::raw(format!("  G923 ")),
        Span::styled("●", ok),
        Span::raw(format!(" {}    ", s.gamepad_name)),
        Span::raw("QUIC"),
        if s.connected { Span::styled("●", ok) } else { Span::styled("●", dead) },
        Span::raw(if s.connected { " CONNECTED" } else { " DISCONNECTED" }),
        Span::raw(format!("  {}Hz  pkts {}", PACKET_HZ, s.packets_sent)),
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
  let mode_name = match s.mode { 0 => "MANUAL", 1 => "REMOTE", 2 => "AUTO", _ => "?" };
  f.render_widget(
    Paragraph::new(vec![
      Line::from(format!("  Speed  {:5.1} km/h  ({:.2} m/s)     Heading  {:5.1}°", s.speed_mps * 3.6, s.speed_mps, s.heading_deg)),
      Line::from(format!("  GPS    {:.6}, {:.6}       Alt  {:.1} m", s.lat, s.lon, s.altitude)),
      Line::from(format!("  IMU acc   x {:+6.2}   y {:+6.2}   z {:+6.2}  m/s²", s.acc[0], s.acc[1], s.acc[2])),
      Line::from(format!("  IMU gyro  x {:+6.3}   y {:+6.3}   z {:+6.3}  rad/s     Mode: {}", s.gyro[0], s.gyro[1], s.gyro[2], mode_name)),
    ]).block(Block::bordered().title(" TELEMETRY ")),
    chunks[3],
  );

  // Footer
  f.render_widget(Paragraph::new(vec![
    Line::from(" G=gear  H=handbrake  Q=quit"),
    Line::from(" R1/L1=shift  Circle=handbrake  Options=e-stop"),
  ]), chunks[4]);
}
