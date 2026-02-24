use rmpv::Value;
use std::collections::HashMap;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

const PROTOCOL_VERSION: &str = "v1.26";

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectionState {
  Disconnected,
  Connected,
  Ready,
  Error,
}

pub struct BeamNGBridge {
  host: String,
  port: u16,
  max_steering_angle: f32,
  ge_stream: Option<TcpStream>,
  veh_stream: Option<TcpStream>,
  state: ConnectionState,
  next_id: u64,
  // Control state
  pub throttle: f32,
  pub brake: f32,
  pub steering: f32,
  pub gear: i64,
  pub handbrake: bool,
}

impl BeamNGBridge {
  pub fn new(host: &str, port: u16, max_steering_angle: f32) -> Self {
    Self {
      host: host.to_string(),
      port,
      max_steering_angle,
      ge_stream: None,
      veh_stream: None,
      state: ConnectionState::Disconnected,
      next_id: 0,
      throttle: 0.0,
      brake: 0.0,
      steering: 0.0,
      gear: 3,
      handbrake: false,
    }
  }

  pub fn state(&self) -> ConnectionState {
    self.state
  }

  pub fn is_ready(&self) -> bool {
    self.state == ConnectionState::Ready
  }

  pub async fn connect(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    log::info!("Connecting to BeamNG at {}:{}", self.host, self.port);
    let stream = TcpStream::connect(format!("{}:{}", self.host, self.port)).await?;
    stream.set_nodelay(true)?;
    self.ge_stream = Some(stream);
    self.state = ConnectionState::Connected;

    // Hello handshake
    let resp = self.send_ge(&Value::Map(vec![
      (Value::from("type"), Value::from("Hello")),
      (Value::from("protocolVersion"), Value::from(PROTOCOL_VERSION)),
    ])).await?;

    let version = resp.as_map().and_then(|m| m.iter().find(|(k, _)| k.as_str() == Some("protocolVersion")))
      .and_then(|(_, v)| v.as_str());
    if version != Some(PROTOCOL_VERSION) {
      self.state = ConnectionState::Error;
      return Err(format!("Protocol mismatch: expected {}, got {:?}", PROTOCOL_VERSION, version).into());
    }
    log::info!("BeamNG hello OK ({})", PROTOCOL_VERSION);
    Ok(())
  }

  pub async fn attach_vehicle(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    if self.state != ConnectionState::Connected {
      return Err("Not connected".into());
    }

    // Get current vehicles
    let resp = self.send_ge(&Value::Map(vec![
      (Value::from("type"), Value::from("GetCurrentVehicles")),
    ])).await?;

    let vehicles = resp.as_map().and_then(|m| m.iter().find(|(k, _)| k.as_str() == Some("vehicles")))
      .and_then(|(_, v)| v.as_map());
    let vid = vehicles.and_then(|m| m.first()).and_then(|(k, _)| k.as_str())
      .ok_or("No vehicles found")?;
    let vid = vid.to_string();
    log::info!("Found vehicle: {}", vid);

    // Start vehicle connection
    let resp = self.send_ge(&Value::Map(vec![
      (Value::from("type"), Value::from("StartVehicleConnection")),
      (Value::from("vid"), Value::from(vid.as_str())),
    ])).await?;

    let veh_port = resp.as_map().and_then(|m| m.iter().find(|(k, _)| k.as_str() == Some("result")))
      .and_then(|(_, v)| v.as_u64())
      .ok_or("No vehicle port in response")?;
    log::info!("Vehicle port: {}", veh_port);

    // Connect to vehicle
    let stream = TcpStream::connect(format!("{}:{}", self.host, veh_port)).await?;
    stream.set_nodelay(true)?;
    self.veh_stream = Some(stream);

    // Vehicle hello
    self.send_veh(&Value::Map(vec![
      (Value::from("type"), Value::from("Hello")),
      (Value::from("protocolVersion"), Value::from(PROTOCOL_VERSION)),
    ])).await?;

    // Disable player input
    self.send_veh(&Value::Map(vec![
      (Value::from("type"), Value::from("QueueLuaCommandVE")),
      (Value::from("chunk"), Value::from("extensions.core_input.setAllPlayerInputDisabled(true)")),
    ])).await?;

    self.state = ConnectionState::Ready;
    log::info!("Vehicle attached and ready");
    Ok(())
  }

  pub async fn send_control(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    if !self.is_ready() {
      return Ok(());
    }
    let steering_normalized = (self.steering / self.max_steering_angle).clamp(-1.0, 1.0);
    self.send_veh(&Value::Map(vec![
      (Value::from("type"), Value::from("Control")),
      (Value::from("steering"), Value::from(steering_normalized as f64)),
      (Value::from("throttle"), Value::from(self.throttle as f64)),
      (Value::from("brake"), Value::from(self.brake as f64)),
      (Value::from("parkingbrake"), Value::from(if self.handbrake { 1.0_f64 } else { 0.0 })),
      (Value::from("gear"), Value::from(self.gear)),
    ])).await?;
    Ok(())
  }

  /// Poll vehicle sensors. Returns (pos, rot_quat, vel) if successful.
  pub async fn poll_state(&mut self) -> Result<VehiclePhysics, Box<dyn std::error::Error + Send + Sync>> {
    if !self.is_ready() {
      return Err("Not ready".into());
    }
    let sensors: HashMap<String, Value> = HashMap::from([
      ("state".to_string(), Value::Map(vec![(Value::from("type"), Value::from("State"))])),
    ]);
    let sensors_val = Value::Map(sensors.into_iter().map(|(k, v)| (Value::from(k), v)).collect());
    let resp = self.send_veh(&Value::Map(vec![
      (Value::from("type"), Value::from("SensorRequest")),
      (Value::from("sensors"), sensors_val),
    ])).await?;

    // Parse: resp.data.state.state.{pos, vel, rotation}
    let data = map_get(&resp, "data").and_then(|d| map_get(d, "state")).and_then(|s| map_get(s, "state"));
    let data = data.ok_or("Missing sensor data")?;

    let pos = parse_vec3(map_get(data, "pos"));
    let vel = parse_vec3(map_get(data, "vel"));
    let rot = parse_vec4(map_get(data, "rotation"));

    Ok(VehiclePhysics { pos, vel, rot })
  }

  pub fn disconnect(&mut self) {
    self.ge_stream = None;
    self.veh_stream = None;
    self.state = ConnectionState::Disconnected;
  }

  // --- Wire protocol ---

  async fn send_ge(&mut self, msg: &Value) -> Result<Value, Box<dyn std::error::Error + Send + Sync>> {
    let stream = self.ge_stream.as_mut().ok_or("No GE connection")?;
    send_recv(stream, msg, &mut self.next_id).await
  }

  async fn send_veh(&mut self, msg: &Value) -> Result<Value, Box<dyn std::error::Error + Send + Sync>> {
    let stream = self.veh_stream.as_mut().ok_or("No vehicle connection")?;
    send_recv(stream, msg, &mut self.next_id).await
  }
}

pub struct VehiclePhysics {
  pub pos: [f64; 3],
  pub vel: [f64; 3],
  pub rot: [f64; 4],
}

// Frame: [u32 big-endian length][msgpack payload]
async fn send_recv(stream: &mut TcpStream, msg: &Value, next_id: &mut u64) -> Result<Value, Box<dyn std::error::Error + Send + Sync>> {
  let id = *next_id;
  *next_id += 1;

  // Inject _id
  let mut map: Vec<(Value, Value)> = if let Value::Map(m) = msg { m.clone() } else { vec![] };
  map.push((Value::from("_id"), Value::from(id)));
  let mut payload = Vec::new();
  rmpv::encode::write_value(&mut payload, &Value::Map(map))?;

  // Send length-prefixed frame
  stream.write_all(&(payload.len() as u32).to_be_bytes()).await?;
  stream.write_all(&payload).await?;
  stream.flush().await?;

  // Receive response
  let mut len_buf = [0u8; 4];
  stream.read_exact(&mut len_buf).await?;
  let len = u32::from_be_bytes(len_buf) as usize;
  let mut buf = vec![0u8; len];
  stream.read_exact(&mut buf).await?;

  let resp: Value = rmpv::decode::read_value(&mut buf.as_slice())?;
  Ok(resp)
}

fn map_get<'a>(val: &'a Value, key: &str) -> Option<&'a Value> {
  val.as_map().and_then(|m| m.iter().find(|(k, _)| k.as_str() == Some(key)).map(|(_, v)| v))
}

fn parse_vec3(val: Option<&Value>) -> [f64; 3] {
  val.and_then(|v| v.as_array()).map(|a| {
    [
      a.first().and_then(|x| x.as_f64()).unwrap_or(0.0),
      a.get(1).and_then(|x| x.as_f64()).unwrap_or(0.0),
      a.get(2).and_then(|x| x.as_f64()).unwrap_or(0.0),
    ]
  }).unwrap_or([0.0; 3])
}

fn parse_vec4(val: Option<&Value>) -> [f64; 4] {
  val.and_then(|v| v.as_array()).map(|a| {
    [
      a.first().and_then(|x| x.as_f64()).unwrap_or(0.0),
      a.get(1).and_then(|x| x.as_f64()).unwrap_or(0.0),
      a.get(2).and_then(|x| x.as_f64()).unwrap_or(0.0),
      a.get(3).and_then(|x| x.as_f64()).unwrap_or(1.0),
    ]
  }).unwrap_or([0.0, 0.0, 0.0, 1.0])
}
