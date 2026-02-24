use gilrs::{Axis, Button, Event, EventType, Gilrs};

const GEAR_NAMES: [&str; 4] = ["P", "R", "N", "D"];

pub struct GamepadState {
  pub steering: f32,
  pub throttle: f32,
  pub brake: f32,
  pub gear: u8,
  pub handbrake: bool,
  shift_up_prev: bool,
  shift_down_prev: bool,
  handbrake_prev: bool,
  max_steer: f32,
  deadzone: f32,
  logged_diagnostics: bool,
}

impl GamepadState {
  pub fn new(max_steering_angle: f32, deadzone: f32) -> Self {
    Self {
      steering: 0.0,
      throttle: 0.0,
      brake: 0.0,
      gear: 3, // Drive
      handbrake: false,
      shift_up_prev: false,
      shift_down_prev: false,
      handbrake_prev: false,
      max_steer: max_steering_angle,
      deadzone,
      logged_diagnostics: false,
    }
  }

  pub fn poll(&mut self, gilrs: &mut Gilrs) {
    while let Some(Event { event, id, .. }) = gilrs.next_event() {
      // Log mapping source on first event so we can verify the G923 is recognized
      if !self.logged_diagnostics {
        let gp = gilrs.gamepad(id);
        log::info!("First event from '{}', mapping: {:?}", gp.name(), gp.mapping_source());
        self.logged_diagnostics = true;
      }

      match event {
        // Right paddle → shift up (P→R→N→D, clamped at D)
        EventType::ButtonPressed(Button::RightTrigger | Button::RightTrigger2, _) => {
          if !self.shift_up_prev {
            self.gear = (self.gear + 1).min(3);
            log::info!("Gear: {}", GEAR_NAMES[self.gear as usize]);
          }
          self.shift_up_prev = true;
        }
        EventType::ButtonReleased(Button::RightTrigger | Button::RightTrigger2, _) => {
          self.shift_up_prev = false;
        }
        // Left paddle → shift down (D→N→R→P, clamped at P)
        EventType::ButtonPressed(Button::LeftTrigger | Button::LeftTrigger2, _) => {
          if !self.shift_down_prev {
            self.gear = self.gear.saturating_sub(1);
            log::info!("Gear: {}", GEAR_NAMES[self.gear as usize]);
          }
          self.shift_down_prev = true;
        }
        EventType::ButtonReleased(Button::LeftTrigger | Button::LeftTrigger2, _) => {
          self.shift_down_prev = false;
        }
        // Circle/B → handbrake toggle
        EventType::ButtonPressed(Button::East, _) => {
          if !self.handbrake_prev {
            self.handbrake = !self.handbrake;
            log::info!("Handbrake: {}", if self.handbrake { "ON" } else { "OFF" });
          }
          self.handbrake_prev = true;
        }
        EventType::ButtonReleased(Button::East, _) => {
          self.handbrake_prev = false;
        }
        // Start/Options → emergency stop
        EventType::ButtonPressed(Button::Start, _) => {
          self.throttle = 0.0;
          self.brake = 1.0;
          log::warn!("E-STOP");
        }
        _ => {}
      }
    }

    // Read axes from first active gamepad
    if let Some((_id, gamepad)) = gilrs.gamepads().next() {
      // Steering wheel (LeftStickX) — natural direction, with deadzone for centering
      let raw_steer = gamepad.value(Axis::LeftStickX);
      self.steering = self.apply_deadzone(raw_steer) * self.max_steer;

      // Throttle pedal (LeftStickY) — normalize to 0..1
      self.throttle = normalize_pedal(gamepad.value(Axis::LeftStickY));

      // Brake pedal (LeftZ) — normalize to 0..1
      self.brake = normalize_pedal(gamepad.value(Axis::LeftZ));
    }
  }

  fn apply_deadzone(&self, value: f32) -> f32 {
    if value.abs() < self.deadzone {
      return 0.0;
    }
    let sign = value.signum();
    sign * (value.abs() - self.deadzone) / (1.0 - self.deadzone)
  }
}

/// Normalize pedal value to 0..1. Handles both 0..1 and -1..1 ranges.
/// If min is negative (driver reports -1..1), maps -1→0, 1→1.
/// If min is ~0 (driver reports 0..1), passes through.
fn normalize_pedal(v: f32) -> f32 {
  if v < 0.0 {
    // -1..1 range: remap to 0..1
    ((v + 1.0) / 2.0).clamp(0.0, 1.0)
  } else {
    v.clamp(0.0, 1.0)
  }
}
