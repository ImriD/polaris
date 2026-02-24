use mavlink::ardupilotmega::MavMessage;
use mavlink::{MavConnection, MavHeader};
use std::time::Duration;

const GEAR_PWM: [u16; 4] = [1000, 1250, 1500, 1750]; // P, R, N, D
const HANDBRAKE_ENGAGED: u16 = 2000;
const HANDBRAKE_RELEASED: u16 = 1000;

#[derive(Clone, Default)]
pub struct Telemetry {
  pub lat: f64,
  pub lon: f64,
  pub alt: f64,
  pub heading_deg: f32,
  pub speed_mps: f32,
  pub acc: [f32; 3],
  pub gyro: [f32; 3],
}

pub struct PixhawkInterface {
  conn: Box<dyn MavConnection<MavMessage> + Send + Sync>,
  target_system: u8,
  target_component: u8,
}

impl PixhawkInterface {
  pub fn connect(addr: &str) -> Result<Self, Box<dyn std::error::Error>> {
    log::info!("Connecting to Pixhawk: {}", addr);
    let conn = mavlink::connect::<MavMessage>(addr)?;

    // Wait for heartbeat
    log::info!("Waiting for heartbeat...");
    let mut target_system = 1u8;
    let mut target_component = 1u8;
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    loop {
      if std::time::Instant::now() > deadline {
        return Err("Heartbeat timeout".into());
      }
      if let Ok((_header, msg)) = conn.recv() {
        if let MavMessage::HEARTBEAT(_) = msg {
          target_system = _header.system_id;
          target_component = _header.component_id;
          log::info!("Heartbeat received (sysid={})", target_system);
          break;
        }
      }
    }

    Ok(Self { conn, target_system, target_component })
  }

  pub fn set_manual_mode(&self) {
    let msg = MavMessage::COMMAND_LONG(mavlink::ardupilotmega::COMMAND_LONG_DATA {
      target_system: self.target_system,
      target_component: self.target_component,
      command: mavlink::ardupilotmega::MavCmd::MAV_CMD_DO_SET_MODE,
      confirmation: 0,
      param1: 1.0, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
      param2: 0.0, // custom_mode = MANUAL for Rover
      param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0,
    });
    let _ = self.conn.send(&MavHeader::default(), &msg);
    log::info!("Sent SET_MODE MANUAL");
  }

  pub fn send_neutral_rc(&self) {
    let msg = MavMessage::RC_CHANNELS_OVERRIDE(mavlink::ardupilotmega::RC_CHANNELS_OVERRIDE_DATA {
      target_system: self.target_system,
      target_component: self.target_component,
      chan1_raw: 1500, // steering center
      chan2_raw: 1000, // brake off
      chan3_raw: 1500, // throttle center (stopped)
      chan4_raw: 1500, // gear center
      chan5_raw: 1000, // handbrake off
      ..Default::default()
    });
    let _ = self.conn.send(&MavHeader::default(), &msg);
  }

  pub fn arm(&self) {
    let msg = MavMessage::COMMAND_LONG(mavlink::ardupilotmega::COMMAND_LONG_DATA {
      target_system: self.target_system,
      target_component: self.target_component,
      command: mavlink::ardupilotmega::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
      confirmation: 0,
      param1: 1.0,     // arm
      param2: 21196.0, // force arm (skip pre-arm checks)
      param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0,
    });
    let _ = self.conn.send(&MavHeader::default(), &msg);
    log::info!("Sent ARM command (force)");
  }

  pub fn request_data_streams(&self, rate_hz: u16) {
    let msg = MavMessage::REQUEST_DATA_STREAM(mavlink::ardupilotmega::REQUEST_DATA_STREAM_DATA {
      target_system: self.target_system,
      target_component: self.target_component,
      req_stream_id: 0, // MAV_DATA_STREAM_ALL
      req_message_rate: rate_hz,
      start_stop: 1,
    });
    let _ = self.conn.send(&MavHeader::default(), &msg);
  }

  pub fn send_rc_override(&self, steering_norm: f32, throttle: f32, brake: f32, gear: u8, handbrake: bool) {
    let steer_pwm = value_to_pwm_centered(steering_norm.clamp(-1.0, 1.0), 1000, 2000, 1500);
    let throttle_pwm = value_to_pwm(throttle.clamp(0.0, 1.0), 1500, 2000);
    let brake_pwm = value_to_pwm(brake.clamp(0.0, 1.0), 1000, 2000);
    let gear_pwm = if (gear as usize) < GEAR_PWM.len() { GEAR_PWM[gear as usize] } else { GEAR_PWM[0] };
    let hb_pwm = if handbrake { HANDBRAKE_ENGAGED } else { HANDBRAKE_RELEASED };

    let msg = MavMessage::RC_CHANNELS_OVERRIDE(mavlink::ardupilotmega::RC_CHANNELS_OVERRIDE_DATA {
      target_system: self.target_system,
      target_component: self.target_component,
      chan1_raw: steer_pwm,
      chan2_raw: brake_pwm,
      chan3_raw: throttle_pwm,
      chan4_raw: gear_pwm,
      chan5_raw: hb_pwm,
      ..Default::default()
    });
    let _ = self.conn.send(&MavHeader::default(), &msg);
  }

  /// Read one message. Returns updated telemetry if relevant message received.
  pub fn recv_telemetry(&self) -> Option<TelemetryUpdate> {
    match self.conn.recv() {
      Ok((_header, msg)) => match msg {
        MavMessage::GLOBAL_POSITION_INT(data) => Some(TelemetryUpdate::Position {
          lat: data.lat as f64 / 1e7,
          lon: data.lon as f64 / 1e7,
          alt: data.alt as f64 / 1000.0,
          heading_deg: data.hdg as f32 / 100.0,
          speed_mps: ((data.vx as f32).powi(2) + (data.vy as f32).powi(2)).sqrt() / 100.0,
        }),
        MavMessage::RAW_IMU(data) => Some(TelemetryUpdate::Imu {
          acc: [data.xacc as f32 / 1000.0, data.yacc as f32 / 1000.0, data.zacc as f32 / 1000.0],
          gyro: [data.xgyro as f32 / 1000.0, data.ygyro as f32 / 1000.0, data.zgyro as f32 / 1000.0],
        }),
        _ => None,
      },
      Err(_) => None,
    }
  }
}

pub enum TelemetryUpdate {
  Position { lat: f64, lon: f64, alt: f64, heading_deg: f32, speed_mps: f32 },
  Imu { acc: [f32; 3], gyro: [f32; 3] },
}

fn value_to_pwm(value: f32, min: u16, max: u16) -> u16 {
  (min as f32 + value * (max - min) as f32) as u16
}

fn value_to_pwm_centered(value: f32, min: u16, max: u16, center: u16) -> u16 {
  if value >= 0.0 {
    (center as f32 + value * (max - center) as f32) as u16
  } else {
    (center as f32 + value * (center - min) as f32) as u16
  }
}
