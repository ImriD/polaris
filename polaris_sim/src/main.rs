mod bridge;

use bridge::{BeamNGBridge, ConnectionState};
use futures::StreamExt;
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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
  env_logger::init();
  let ctx = r2r::Context::create()?;
  let mut node = r2r::Node::create(ctx, "beamng_bridge", "")?;

  let host = "fastgpu".to_string();
  let port = 64256u16;
  let max_steering_angle = 0.5236f32;

  let sensor_qos = QosProfile::sensor_data();
  let reliable = QosProfile::default();

  // Subscribe to /cmd/* (filtered commands from vehicle_control_node)
  let steer_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/steering", reliable.clone())?;
  let throt_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/throttle", reliable.clone())?;
  let brake_sub = node.subscribe::<r2r::std_msgs::msg::Float32>("/cmd/brake", reliable.clone())?;
  let gear_sub = node.subscribe::<r2r::std_msgs::msg::UInt8>("/cmd/gear", reliable)?;
  let hbrake_sub = node.subscribe::<r2r::std_msgs::msg::Bool>("/cmd/handbrake", QosProfile::default())?;

  let state_pub = node.create_publisher::<r2r::polaris_msgs::msg::VehicleState>("/vehicle/state", sensor_qos)?;

  let cmd: Arc<Mutex<CmdState>> = Arc::new(Mutex::new(CmdState::default()));

  // Handle /cmd/*
  let c = cmd.clone();
  tokio::spawn(async move {
    steer_sub.for_each(|msg| { c.lock().unwrap().steering = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::spawn(async move {
    throt_sub.for_each(|msg| { c.lock().unwrap().throttle = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::spawn(async move {
    brake_sub.for_each(|msg| { c.lock().unwrap().brake = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::spawn(async move {
    gear_sub.for_each(|msg| { c.lock().unwrap().gear = msg.data; std::future::ready(()) }).await;
  });
  let c = cmd.clone();
  tokio::spawn(async move {
    hbrake_sub.for_each(|msg| { c.lock().unwrap().handbrake = msg.data; std::future::ready(()) }).await;
  });

  // Bridge task
  let c = cmd.clone();
  tokio::spawn(async move {
    let mut bridge = BeamNGBridge::new(&host, port, max_steering_angle);

    loop {
      if bridge.state() != ConnectionState::Ready {
        bridge.disconnect();
        if let Err(e) = bridge.connect().await {
          log::warn!("Connect failed: {e}");
          tokio::time::sleep(std::time::Duration::from_secs(2)).await;
          continue;
        }
        if let Err(e) = bridge.attach_vehicle().await {
          log::warn!("Attach failed: {e}");
          tokio::time::sleep(std::time::Duration::from_secs(2)).await;
          continue;
        }
      }

      // Apply latest control inputs
      {
        let cmd = c.lock().unwrap();
        bridge.throttle = cmd.throttle;
        bridge.brake = cmd.brake;
        bridge.steering = cmd.steering;
        bridge.gear = cmd.gear as i64;
        bridge.handbrake = cmd.handbrake;
      }

      if let Err(e) = bridge.send_control().await {
        log::warn!("Control error: {}", e);
        bridge.disconnect();
        continue;
      }

      match bridge.poll_state().await {
        Ok(physics) => {
          let cmd = c.lock().unwrap();
          let speed = (physics.vel[0].powi(2) + physics.vel[1].powi(2) + physics.vel[2].powi(2)).sqrt();
          let rot = physics.rot;
          let heading = (2.0 * (rot[3] * rot[2] + rot[0] * rot[1]))
            .atan2(1.0 - 2.0 * (rot[1].powi(2) + rot[2].powi(2)))
            .to_degrees();

          let mut vs = r2r::polaris_msgs::msg::VehicleState::default();
          vs.steering_angle = cmd.steering;
          vs.throttle_position = cmd.throttle;
          vs.brake_pressure = cmd.brake;
          vs.gear_current = cmd.gear;
          vs.handbrake_engaged = cmd.handbrake;
          drop(cmd);
          vs.speed_mps = speed as f32;
          vs.heading_deg = heading as f32;
          vs.lat = physics.pos[1];
          vs.lon = physics.pos[0];
          vs.mode = 1;
          let _ = state_pub.publish(&vs);
        }
        Err(e) => {
          log::warn!("Poll error: {}", e);
          bridge.disconnect();
          continue;
        }
      }

      tokio::time::sleep(std::time::Duration::from_millis(20)).await;
    }
  });

  log::info!("BeamNG bridge node started");

  loop {
    node.spin_once(std::time::Duration::from_millis(10));
    tokio::time::sleep(std::time::Duration::from_millis(1)).await;
  }
}
