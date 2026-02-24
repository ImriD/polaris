use gilrs::Gilrs;

#[path = "../gamepad.rs"]
mod gamepad;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  env_logger::init();
  let ctx = r2r::Context::create()?;
  let mut node = r2r::Node::create(ctx, "gamepad", "")?;

  let reliable = r2r::QosProfile::default();
  let steer_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/steering", reliable.clone())?;
  let throt_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/throttle", reliable.clone())?;
  let brake_pub = node.create_publisher::<r2r::std_msgs::msg::Float32>("/input/brake", reliable.clone())?;
  let gear_pub = node.create_publisher::<r2r::std_msgs::msg::UInt8>("/input/gear", reliable.clone())?;
  let hbrake_pub = node.create_publisher::<r2r::std_msgs::msg::Bool>("/input/handbrake", reliable)?;

  let mut gilrs = Gilrs::new().map_err(|e| format!("gilrs init failed: {}", e))?;
  if let Some((_id, gp)) = gilrs.gamepads().next() {
    log::info!("Connected: {} (mapping: {:?})", gp.name(), gp.mapping_source());
  } else {
    log::warn!("No gamepad found — run tui_dashboard for keyboard control");
  }

  let mut state = gamepad::GamepadState::new(0.5236, 0.05);

  log::info!("Gamepad node ready");

  // 50Hz main loop
  loop {
    state.poll(&mut gilrs);

    let _ = steer_pub.publish(&r2r::std_msgs::msg::Float32 { data: state.steering });
    let _ = throt_pub.publish(&r2r::std_msgs::msg::Float32 { data: state.throttle });
    let _ = brake_pub.publish(&r2r::std_msgs::msg::Float32 { data: state.brake });
    let _ = gear_pub.publish(&r2r::std_msgs::msg::UInt8 { data: state.gear });
    let _ = hbrake_pub.publish(&r2r::std_msgs::msg::Bool { data: state.handbrake });

    node.spin_once(std::time::Duration::from_millis(1));
    tokio::time::sleep(std::time::Duration::from_millis(20)).await;
  }
}
