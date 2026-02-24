mod pixhawk;

use futures::StreamExt;
use pixhawk::{PixhawkInterface, TelemetryUpdate};
use r2r::QosProfile;
use std::sync::{Arc, Mutex};

struct CmdState {
  steering: f32,
  throttle: f32,
  brake: f32,
  gear: u8,
  handbrake: bool,
}

impl Default for CmdState {
  fn default() -> Self {
    Self { steering: 0.0, throttle: 0.0, brake: 0.0, gear: 3, handbrake: false }
  }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  let local = tokio::task::LocalSet::new();
  local.run_until(async_main()).await
}

async fn async_main() -> Result<(), Box<dyn std::error::Error>> {
  env_logger::init();
  let ctx = r2r::Context::create()?;
  let mut node = r2r::Node::create(ctx, "pixhawk_bridge", "")?;

  let pixhawk_addr = std::env::var("PIXHAWK_ADDR").unwrap_or_else(|_| "serial:/dev/ttyUSB0:921600".into());
  let max_steering_angle = 7.854f32; // 450 degrees in radians
  let telemetry_rate = 10u16;

  let sensor_qos = QosProfile::sensor_data();
  let reliable = QosProfile::default();

  // Subscribe to /cmd/* (filtered commands from vehicle_control_node)
  let steer_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/steering", reliable.clone())?;
  let throt_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/throttle", reliable.clone())?;
  let brake_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/brake", reliable.clone())?;
  let gear_sub = node.subscribe::<r2r::std_msgs::msg::UInt8>("/cmd/gear", reliable)?;
  let hbrake_sub = node.subscribe::<r2r::std_msgs::msg::Bool>("/cmd/handbrake", QosProfile::default())?;

  let state_pub = node.create_publisher::<r2r::polaris_msgs::msg::VehicleState>("/vehicle/state", sensor_qos.clone())?;
  let imu_pub = node.create_publisher::<r2r::sensor_msgs::msg::Imu>("/vehicle/imu", sensor_qos.clone())?;
  let gps_pub = node.create_publisher::<r2r::sensor_msgs::msg::NavSatFix>("/vehicle/gps", sensor_qos)?;

  let mut connect_timer = node.create_wall_timer(std::time::Duration::from_secs(2))?;

  let cmd: Arc<Mutex<CmdState>> = Arc::new(Mutex::new(CmdState::default()));
  let pixhawk: Arc<Mutex<Option<PixhawkInterface>>> = Arc::new(Mutex::new(None));

  // Handle /cmd/* — store latest values
  let c = cmd.clone();
  tokio::task::spawn_local(async move {
    steer_sub.for_each(|msg| { c.lock().unwrap().steering = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::task::spawn_local(async move {
    throt_sub.for_each(|msg| { c.lock().unwrap().throttle = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::task::spawn_local(async move {
    brake_sub.for_each(|msg| { c.lock().unwrap().brake = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::task::spawn_local(async move {
    gear_sub.for_each(|msg| { c.lock().unwrap().gear = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::task::spawn_local(async move {
    hbrake_sub.for_each(|msg| { c.lock().unwrap().handbrake = msg.data; std::future::ready(()) }).await;
  });

  // Connection retry at 2Hz
  let px = pixhawk.clone();
  let addr = pixhawk_addr.clone();
  tokio::task::spawn_local(async move {
    loop {
      let _ = connect_timer.tick().await;
      let mut guard = px.lock().unwrap();
      if guard.is_some() { continue; }
      log::info!("Connecting to Pixhawk...");
      match PixhawkInterface::connect(&addr) {
        Ok(iface) => {
          iface.request_data_streams(telemetry_rate);
          iface.set_manual_mode();
          iface.arm();
          *guard = Some(iface);
          log::info!("Pixhawk connected, armed, telemetry streaming");
        }
        Err(e) => log::warn!("Pixhawk connection failed: {}", e),
      }
    }
  });

  // Telemetry reader thread (blocking MAVLink recv)
  let px = pixhawk.clone();
  let c = cmd.clone();
  let max_steer = max_steering_angle;
  std::thread::spawn(move || {
    loop {
      let guard = px.lock().unwrap();
      let Some(ref iface) = *guard else {
        drop(guard);
        std::thread::sleep(std::time::Duration::from_millis(100));
        continue;
      };
      match iface.recv_telemetry() {
        Some(TelemetryUpdate::Position { lat, lon, alt, heading_deg, speed_mps }) => {
          let cmd = c.lock().unwrap();
          iface.send_rc_override(cmd.steering / max_steer, cmd.throttle, cmd.brake, cmd.gear, cmd.handbrake);

          let mut vs = r2r::polaris_msgs::msg::VehicleState::default();
          vs.steering_angle = cmd.steering;
          vs.throttle_position = cmd.throttle;
          vs.brake_pressure = cmd.brake;
          vs.gear_current = cmd.gear;
          vs.handbrake_engaged = cmd.handbrake;
          drop(cmd);
          vs.speed_mps = speed_mps;
          vs.heading_deg = heading_deg;
          vs.lat = lat;
          vs.lon = lon;
          vs.mode = 1;
          let _ = state_pub.publish(&vs);

          let mut gps = r2r::sensor_msgs::msg::NavSatFix::default();
          gps.header.frame_id = "gps".to_string();
          gps.latitude = lat;
          gps.longitude = lon;
          gps.altitude = alt;
          gps.status.status = 0;
          gps.status.service = 1;
          gps.position_covariance_type = 2;
          gps.position_covariance = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0];
          let _ = gps_pub.publish(&gps);
        }
        Some(TelemetryUpdate::Imu { acc, gyro }) => {
          let cmd = c.lock().unwrap();
          iface.send_rc_override(cmd.steering / max_steer, cmd.throttle, cmd.brake, cmd.gear, cmd.handbrake);
          drop(cmd);

          let mut imu = r2r::sensor_msgs::msg::Imu::default();
          imu.header.frame_id = "imu".to_string();
          imu.linear_acceleration.x = acc[0] as f64;
          imu.linear_acceleration.y = acc[1] as f64;
          imu.linear_acceleration.z = acc[2] as f64;
          imu.angular_velocity.x = gyro[0] as f64;
          imu.angular_velocity.y = gyro[1] as f64;
          imu.angular_velocity.z = gyro[2] as f64;
          imu.orientation.w = 1.0;
          imu.orientation_covariance = vec![-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
          let _ = imu_pub.publish(&imu);
        }
        None => {}
      }
      drop(guard);
    }
  });

  log::info!("Pixhawk bridge node started");

  loop {
    node.spin_once(std::time::Duration::from_millis(10));
    tokio::time::sleep(std::time::Duration::from_millis(1)).await;
  }
}
