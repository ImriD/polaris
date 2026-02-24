use futures::StreamExt;
use r2r::QosProfile;
use std::sync::{Arc, Mutex};

struct Actuators {
  steering: f32,
  throttle: f32,
  brake: f32,
  gear: u8,
  handbrake: bool,
}

impl Default for Actuators {
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
  let mut node = r2r::Node::create(ctx, "vehicle_control", "")?;

  let state = Arc::new(Mutex::new(Actuators::default()));
  let qos = QosProfile::default();

  // Subscribe /input/* (operator raw)
  let subs: Vec<_> = ["/input/steering", "/input/throttle", "/input/brake"].iter()
    .map(|t| node.subscribe::<r2r::std_msgs::msg::Float32>(t, qos.clone()).unwrap())
    .collect();
  let gear_sub = node.subscribe::<r2r::std_msgs::msg::UInt8>("/input/gear", qos.clone())?;
  let hbrake_sub = node.subscribe::<r2r::std_msgs::msg::Bool>("/input/handbrake", qos.clone())?;

  // Publish /cmd/* (forwarded)
  let cmd_steer = node.create_publisher::<r2r::std_msgs::msg::Float32>("/cmd/steering", qos.clone())?;
  let cmd_throt = node.create_publisher::<r2r::std_msgs::msg::Float32>("/cmd/throttle", qos.clone())?;
  let cmd_brake = node.create_publisher::<r2r::std_msgs::msg::Float32>("/cmd/brake", qos.clone())?;
  let cmd_gear = node.create_publisher::<r2r::std_msgs::msg::UInt8>("/cmd/gear", qos.clone())?;
  let cmd_hbrake = node.create_publisher::<r2r::std_msgs::msg::Bool>("/cmd/handbrake", qos)?;

  let mut timer = node.create_wall_timer(std::time::Duration::from_millis(20))?; // 50Hz

  // /input/steering
  let st = state.clone();
  let mut steer_sub = subs.into_iter();
  let s = steer_sub.next().unwrap();
  tokio::task::spawn_local(async move {
    s.for_each(|msg| { st.lock().unwrap().steering = msg.data; std::future::ready(()) }).await;
  });

  // /input/throttle
  let st = state.clone();
  let s = steer_sub.next().unwrap();
  tokio::task::spawn_local(async move {
    s.for_each(|msg| { st.lock().unwrap().throttle = msg.data; std::future::ready(()) }).await;
  });

  // /input/brake
  let st = state.clone();
  let s = steer_sub.next().unwrap();
  tokio::task::spawn_local(async move {
    s.for_each(|msg| { st.lock().unwrap().brake = msg.data; std::future::ready(()) }).await;
  });

  // /input/gear
  let st = state.clone();
  tokio::task::spawn_local(async move {
    gear_sub.for_each(|msg| { st.lock().unwrap().gear = msg.data; std::future::ready(()) }).await;
  });

  // /input/handbrake
  let st = state.clone();
  tokio::task::spawn_local(async move {
    hbrake_sub.for_each(|msg| { st.lock().unwrap().handbrake = msg.data; std::future::ready(()) }).await;
  });

  // Forward at 50Hz
  let st = state.clone();
  tokio::task::spawn_local(async move {
    loop {
      let _ = timer.tick().await;
      let a = st.lock().unwrap();
      let _ = cmd_steer.publish(&r2r::std_msgs::msg::Float32 { data: a.steering });
      let _ = cmd_throt.publish(&r2r::std_msgs::msg::Float32 { data: a.throttle });
      let _ = cmd_brake.publish(&r2r::std_msgs::msg::Float32 { data: a.brake });
      let _ = cmd_gear.publish(&r2r::std_msgs::msg::UInt8 { data: a.gear });
      let _ = cmd_hbrake.publish(&r2r::std_msgs::msg::Bool { data: a.handbrake });
    }
  });

  log::info!("Vehicle control started (5 actuators, passthrough)");

  loop {
    node.spin_once(std::time::Duration::from_millis(10));
    tokio::time::sleep(std::time::Duration::from_millis(1)).await;
  }
}
