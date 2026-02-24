"""Launch SITL stack: pixhawk_bridge_node (UDP to SITL) + vehicle_control_node."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
  workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  bin_dir = os.path.join(workspace, "target", "release")

  return LaunchDescription([
    ExecuteProcess(
      cmd=[os.path.join(bin_dir, "pixhawk_bridge_node")],
      name="pixhawk_bridge_node",
      output="screen",
      additional_env={"PIXHAWK_ADDR": "udp:127.0.0.1:5762"},
    ),
    ExecuteProcess(
      cmd=[os.path.join(bin_dir, "vehicle_control_node")],
      name="vehicle_control_node",
      output="screen",
    ),
  ])
