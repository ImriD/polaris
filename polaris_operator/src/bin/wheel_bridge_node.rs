use futures::StreamExt;
use quinn::rustls::pki_types::{CertificateDer, PrivatePkcs8KeyDer};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

const LISTEN_PORT: u16 = 9876;

struct WheelState {
  steering: f32,
  throttle: f32,
  brake: f32,
  gear: u8,
  handbrake: bool,
}

impl Default for WheelState {
  fn default() -> Self {
    Self { steering: 0.0, throttle: 0.0, brake: 0.0, gear: 3, handbrake: false }
  }
}

fn parse_packet(buf: &[u8; 14]) -> WheelState {
  WheelState {
    steering: f32::from_le_bytes(buf[0..4].try_into().unwrap()),
    throttle: f32::from_le_bytes(buf[4..8].try_into().unwrap()),
    brake: f32::from_le_bytes(buf[8..12].try_into().unwrap()),
    gear: buf[12],
    handbrake: buf[13] != 0,
  }
}

#[derive(Default, Clone)]
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
  fn to_bytes(&self) -> [u8; 72] {
    let mut buf = [0u8; 72];
    buf[0..4].copy_from_slice(&self.speed_mps.to_le_bytes());
    buf[4..8].copy_from_slice(&self.heading_deg.to_le_bytes());
    buf[8..16].copy_from_slice(&self.lat.to_le_bytes());
    buf[16..24].copy_from_slice(&self.lon.to_le_bytes());
    buf[24..28].copy_from_slice(&self.safe_steering.to_le_bytes());
    buf[28..32].copy_from_slice(&self.safe_throttle.to_le_bytes());
    buf[32..36].copy_from_slice(&self.safe_brake.to_le_bytes());
    buf[36] = self.safe_gear;
    buf[37] = self.safe_handbrake;
    buf[38] = self.gear_current;
    buf[39] = self.mode;
    buf[40..48].copy_from_slice(&self.altitude.to_le_bytes());
    buf[48..52].copy_from_slice(&self.acc_x.to_le_bytes());
    buf[52..56].copy_from_slice(&self.acc_y.to_le_bytes());
    buf[56..60].copy_from_slice(&self.acc_z.to_le_bytes());
    buf[60..64].copy_from_slice(&self.gyro_x.to_le_bytes());
    buf[64..68].copy_from_slice(&self.gyro_y.to_le_bytes());
    buf[68..72].copy_from_slice(&self.gyro_z.to_le_bytes());
    buf
  }
}

fn load_or_generate_cert() -> Result<(CertificateDer<'static>, PrivatePkcs8KeyDer<'static>), Box<dyn std::error::Error>> {
  let dir = std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".config/polaris");
  std::fs::create_dir_all(&dir)?;
  let cert_path = dir.join("cert.der");
  let key_path = dir.join("key.der");
  if cert_path.exists() && key_path.exists() {
    let cert = CertificateDer::from(std::fs::read(&cert_path)?);
    let key = PrivatePkcs8KeyDer::from(std::fs::read(&key_path)?);
    log::info!("Loaded TLS cert from {}", cert_path.display());
    return Ok((cert, key));
  }
  let cert = rcgen::generate_simple_self_signed(vec!["fastgpu.local".to_string(), "localhost".to_string()])?;
  let cert_der = CertificateDer::from(cert.cert);
  let key_der = PrivatePkcs8KeyDer::from(cert.signing_key.serialize_der());
  std::fs::write(&cert_path, &cert_der)?;
  std::fs::write(&key_path, key_der.secret_pkcs8_der())?;
  log::info!("Generated self-signed TLS cert at {}", cert_path.display());
  Ok((cert_der, key_der))
}

async fn handle_connection(conn: quinn::Connection, wheel_state: Arc<Mutex<WheelState>>,
  vehicle_down: Arc<Mutex<VehicleDown>>, tcp_rx_count: Arc<AtomicU64>) {
  let addr = conn.remote_address();
  log::info!("QUIC wheel connected from {addr}");
  let Ok((mut send, mut recv)) = conn.accept_bi().await else {
    log::warn!("accept_bi failed for {addr}");
    return;
  };
  let mut buf = [0u8; 14];
  loop {
    if recv.read_exact(&mut buf).await.is_err() { break; }
    *wheel_state.lock().unwrap() = parse_packet(&buf);
    tcp_rx_count.fetch_add(1, Ordering::Relaxed);
    let telemetry = vehicle_down.lock().unwrap().to_bytes();
    if send.write_all(&telemetry).await.is_err() { break; }
  }
  log::warn!("QUIC wheel disconnected from {addr}");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  env_logger::init();
  let ctx = r2r::Context::create()?;
  let mut node = r2r::Node::create(ctx, "wheel_bridge", "")?;

  let reliable = r2r::QosProfile::default();
  let steer_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/steering", reliable.clone())?;
  let throt_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/throttle", reliable.clone())?;
  let brake_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/brake", reliable.clone())?;
  let gear_pub = node.create_publisher::<r2r::std_msgs::msg::UInt8>("/input/gear", reliable.clone())?;
  let hbrake_pub = node.create_publisher::<r2r::std_msgs::msg::Bool>("/input/handbrake", reliable.clone())?;

  // Subscribe to /vehicle/state
  let vehicle_down = Arc::new(Mutex::new(VehicleDown::default()));
  let vd = vehicle_down.clone();
  let state_sub = node.subscribe::<r2r::polaris_msgs::msg::VehicleState>("/vehicle/state", reliable.clone())?;
  tokio::spawn(state_sub.for_each(move |msg| {
    let mut v = vd.lock().unwrap();
    v.speed_mps = msg.speed_mps;
    v.heading_deg = msg.heading_deg;
    v.lat = msg.lat;
    v.lon = msg.lon;
    v.gear_current = msg.gear_current;
    v.mode = msg.mode;
    std::future::ready(())
  }));

  // Subscribe to /cmd/* for safe actuator values
  let vd = vehicle_down.clone();
  let cmd_steer_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/steering", reliable.clone())?;
  tokio::spawn(cmd_steer_sub.for_each(move |msg| { vd.lock().unwrap().safe_steering = msg.data; std::future::ready(()) }));

  let vd = vehicle_down.clone();
  let cmd_throt_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/throttle", reliable.clone())?;
  tokio::spawn(cmd_throt_sub.for_each(move |msg| { vd.lock().unwrap().safe_throttle = msg.data; std::future::ready(()) }));

  let vd = vehicle_down.clone();
  let cmd_brake_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/brake", reliable.clone())?;
  tokio::spawn(cmd_brake_sub.for_each(move |msg| { vd.lock().unwrap().safe_brake = msg.data; std::future::ready(()) }));

  let vd = vehicle_down.clone();
  let cmd_gear_sub = node.subscribe::<r2r::std_msgs::msg::UInt8>("/cmd/gear", reliable.clone())?;
  tokio::spawn(cmd_gear_sub.for_each(move |msg| { vd.lock().unwrap().safe_gear = msg.data; std::future::ready(()) }));

  let vd = vehicle_down.clone();
  let cmd_hbrake_sub = node.subscribe::<r2r::std_msgs::msg::Bool>("/cmd/handbrake", reliable.clone())?;
  tokio::spawn(cmd_hbrake_sub.for_each(move |msg| { vd.lock().unwrap().safe_handbrake = msg.data as u8; std::future::ready(()) }));

  // Subscribe to /vehicle/imu (sensor_msgs/Imu)
  let vd = vehicle_down.clone();
  let imu_sub = node.subscribe::<r2r::sensor_msgs::msg::Imu>("/vehicle/imu", reliable.clone())?;
  tokio::spawn(imu_sub.for_each(move |msg| {
    let mut v = vd.lock().unwrap();
    v.acc_x = msg.linear_acceleration.x as f32;
    v.acc_y = msg.linear_acceleration.y as f32;
    v.acc_z = msg.linear_acceleration.z as f32;
    v.gyro_x = msg.angular_velocity.x as f32;
    v.gyro_y = msg.angular_velocity.y as f32;
    v.gyro_z = msg.angular_velocity.z as f32;
    std::future::ready(())
  }));

  // Subscribe to /vehicle/gps (sensor_msgs/NavSatFix)
  let vd = vehicle_down.clone();
  let gps_sub = node.subscribe::<r2r::sensor_msgs::msg::NavSatFix>("/vehicle/gps", reliable)?;
  tokio::spawn(gps_sub.for_each(move |msg| {
    vd.lock().unwrap().altitude = msg.altitude;
    std::future::ready(())
  }));

  let wheel_state = Arc::new(Mutex::new(WheelState::default()));
  let tcp_rx_count = Arc::new(AtomicU64::new(0));
  let pub_count = Arc::new(AtomicU64::new(0));

  // QUIC server — accepts connections, bidirectional stream
  let (cert, key) = load_or_generate_cert()?;
  let mut transport = quinn::TransportConfig::default();
  transport.keep_alive_interval(Some(Duration::from_secs(5)));
  transport.max_idle_timeout(Some(Duration::from_secs(30).try_into()?));
  let mut server_config = quinn::ServerConfig::with_single_cert(vec![cert], key.into())?;
  server_config.transport_config(Arc::new(transport));
  let endpoint = quinn::Endpoint::server(server_config, format!("0.0.0.0:{LISTEN_PORT}").parse()?)?;
  log::info!("Listening for QUIC wheel on :{LISTEN_PORT}");

  let ws = wheel_state.clone();
  let vd = vehicle_down.clone();
  let tcp_rx = tcp_rx_count.clone();
  tokio::spawn(async move {
    while let Some(incoming) = endpoint.accept().await {
      match incoming.await {
        Ok(conn) => {
          tokio::spawn(handle_connection(conn, ws.clone(), vd.clone(), tcp_rx.clone()));
        }
        Err(e) => log::warn!("QUIC accept error: {e}"),
      }
    }
  });

  // Publish at 50Hz
  log::info!("Wheel bridge node ready");
  let mut last_pub = std::time::Instant::now();
  let mut last_diag = std::time::Instant::now();

  // Diagnostic logger — print rates every 5 seconds
  let rx = tcp_rx_count.clone();
  let pc = pub_count.clone();

  loop {
    node.spin_once(Duration::from_millis(1));

    if last_pub.elapsed() >= Duration::from_millis(20) {
      let s = wheel_state.lock().unwrap();
      let _ = steer_pub.publish(&r2r::std_msgs::msg::Float32 { data: s.steering });
      let _ = throt_pub.publish(&r2r::std_msgs::msg::Float32 { data: s.throttle });
      let _ = brake_pub.publish(&r2r::std_msgs::msg::Float32 { data: s.brake });
      let _ = gear_pub.publish(&r2r::std_msgs::msg::UInt8 { data: s.gear });
      let _ = hbrake_pub.publish(&r2r::std_msgs::msg::Bool { data: s.handbrake });
      pc.fetch_add(1, Ordering::Relaxed);
      last_pub = std::time::Instant::now();
    }

    if last_diag.elapsed() >= Duration::from_secs(5) {
      let rx_n = rx.swap(0, Ordering::Relaxed);
      let pub_n = pc.swap(0, Ordering::Relaxed);
      log::info!("diag: QUIC rx {:.0} Hz, ROS pub {:.0} Hz", rx_n as f64 / 5.0, pub_n as f64 / 5.0);
      last_diag = std::time::Instant::now();
    }

    tokio::time::sleep(Duration::from_millis(1)).await;
  }
}
