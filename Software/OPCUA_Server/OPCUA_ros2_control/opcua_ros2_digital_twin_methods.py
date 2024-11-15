import sys
import rclpy
from sensor_msgs.msg import JointState
from pathlib import Path
from asyncua import uamethod, ua
from rclpy.executors import SingleThreadedExecutor
import threading
import subprocess
import os
import asyncio
import psutil

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_ros2_nodes.ros2_simulation_joint_state_node import (
    ros2_simulation_joint_state_subscriber,
)
from OPCUA_utils.opcua_paths import opcua_paths
from OPCUA_config.ros2_configuration import ros2_configuration
from OPCUA_ros2_nodes.ros2_digital_twin import ros2_digital_twin
from OPCUA_ros2_nodes.ros2_real_robot_pose import real_robot_cartesian_pose_subscriber
import time

import signal


class ros2_digital_twin_methods:

    def __init__(self):
        self.digital_twin_node = None
        self.digital_twin_executor = None
        self.digital_twin_thread = None

    def _start_digital_twin(self):
        if not rclpy.ok():
            rclpy.init()

        self.digital_twin_node = ros2_digital_twin()
        self.digital_twin_executor = SingleThreadedExecutor()
        self.digital_twin_executor.add_node(self.digital_twin_node)
        self.digital_twin_executor_thread = threading.Thread(
            target=self.digital_twin_executor.spin
        )
        self.digital_twin_executor_thread.start()

    @uamethod
    async def launch_ros2_digital_twin(self, parent):
        self._start_digital_twin()
        return "ROS 2 Digital Twin started successfully."

    @uamethod
    async def shutdown_ros2_digital_twin(self, parent):
        if self.digital_twin_executor:
            self.digital_twin_executor.shutdown()
        if self.digital_twin_node:
            self.digital_twin_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "ROS 2 Digital Twin shutdown successfully."
