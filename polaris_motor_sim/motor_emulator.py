#!/usr/bin/env python3
"""CubeMars AK80-64 motor emulator — receives MIT-mode CAN commands, displays rotating wheel.

Run with --test to use WASD keyboard control instead of CAN bus.
  A/D: rotate left/right
  W/S: increase/decrease rotation speed
"""

import sys
import math
import threading

import pygame

MOTOR_ID = 0x01

# AK80-64 parameter ranges
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -45.0, 45.0
T_MIN, T_MAX = -18.0, 18.0

# Special commands (8 bytes each)
ENTER_CMD = bytes([0xFF] * 7 + [0xFC])
EXIT_CMD  = bytes([0xFF] * 7 + [0xFD])
ZERO_CMD  = bytes([0xFF] * 7 + [0xFE])


def uint_to_float(x, x_min, x_max, bits):
  return x * (x_max - x_min) / ((1 << bits) - 1) + x_min


def float_to_uint(x, x_min, x_max, bits):
  x = max(x_min, min(x_max, x))
  return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def decode_command(data):
  """Unpack MIT-mode command: position(16), velocity(12), kp(12), kd(12), torque(12)."""
  pos = (data[0] << 8) | data[1]
  vel = (data[2] << 4) | (data[3] >> 4)
  t   = ((data[6] & 0x0F) << 8) | data[7]
  return (
    uint_to_float(pos, P_MIN, P_MAX, 16),
    uint_to_float(vel, V_MIN, V_MAX, 12),
    uint_to_float(t, T_MIN, T_MAX, 12),
  )


def encode_response(position, velocity, current):
  """Pack 6-byte feedback: id(8), position(16), velocity(12), current(12)."""
  p = float_to_uint(position, P_MIN, P_MAX, 16)
  v = float_to_uint(velocity, V_MIN, V_MAX, 12)
  c = float_to_uint(current, T_MIN, T_MAX, 12)
  return bytes([
    MOTOR_ID,
    (p >> 8) & 0xFF, p & 0xFF,
    (v >> 4) & 0xFF,
    ((v & 0x0F) << 4) | ((c >> 8) & 0x0F),
    c & 0xFF,
  ])


class Motor:
  def __init__(self):
    self.active = False
    self.position = 0.0
    self.velocity = 0.0

  def handle(self, data, bus):
    if data == ENTER_CMD:
      self.active = True
      self._respond(bus)
      return
    if data == EXIT_CMD:
      self.active = False
      self._respond(bus)
      return
    if data == ZERO_CMD:
      self.position = 0.0
      self._respond(bus)
      return
    if not self.active:
      return
    pos, vel, torque = decode_command(data)
    self.position = pos
    self.velocity = vel
    self._respond(bus)

  def _respond(self, bus):
    import can
    bus.send(can.Message(
      arbitration_id=MOTOR_ID,
      data=encode_response(self.position, self.velocity, 0.0),
      is_extended_id=False,
    ))


def can_listener(motor, bus):
  for msg in bus:
    if msg.arbitration_id == MOTOR_ID and len(msg.data) == 8:
      motor.handle(bytes(msg.data), bus)


def main():
  test_mode = '--test' in sys.argv

  bus = None
  motor = Motor()
  motor.active = True  # always active in test mode

  if not test_mode:
    import subprocess
    subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'], check=True)
    import can
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    motor.active = False
    threading.Thread(target=can_listener, args=(motor, bus), daemon=True).start()

  pygame.init()
  if test_mode:
    screen = pygame.display.set_mode((800, 480))
    pygame.display.set_caption('Motor Emulator (WASD)')
  else:
    screen = pygame.display.set_mode((800, 480), pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
  clock = pygame.time.Clock()
  cx, cy, radius = 400, 240, 200
  step = 0.05  # radians per frame while key held

  while True:
    for event in pygame.event.get():
      if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
        if bus:
          bus.shutdown()
        pygame.quit()
        sys.exit()

    if test_mode:
      keys = pygame.key.get_pressed()
      if keys[pygame.K_a]:
        motor.position -= step
      if keys[pygame.K_d]:
        motor.position += step
      if keys[pygame.K_w]:
        step = min(step + 0.005, 0.5)
      if keys[pygame.K_s]:
        step = max(step - 0.005, 0.01)

    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (255, 255, 255), (cx, cy), radius, 2)
    a = motor.position
    pygame.draw.line(screen, (255, 255, 255), (cx, cy), (cx + radius * math.sin(a), cy - radius * math.cos(a)), 2)
    pygame.display.flip()
    clock.tick(60)


if __name__ == '__main__':
  main()
