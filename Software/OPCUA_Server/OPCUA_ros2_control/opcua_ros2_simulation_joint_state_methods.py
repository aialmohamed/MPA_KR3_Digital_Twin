import sys
import rclpy
from pathlib import Path
from asyncua import uamethod
from rclpy.executors import SingleThreadedExecutor
import threading

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_ros2_nodes.ros2_simulation_joint_state_node import (
    ros2_simulation_joint_state_subscriber,
)

import time



class ros2_simulation_joint_state_methods:
    def __init__(self, joint_state_server_variable):
            self.joint_state_server_variable = joint_state_server_variable
            self.subscriber_node = None
            self.executor = None
            self.joint_state_data = None


    def ros2_setup(self):
        if not rclpy.ok():
            rclpy.init()
        self.subscriber_node = ros2_simulation_joint_state_subscriber(
            self.joint_state_server_variable
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.subscriber_node)
        self.ros_thread = threading.Thread(target=self.spin_with_delay)
        self.ros_thread.start()

    def spin_with_delay(self):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.5)

    @uamethod
    async def subscribe_to_joint_state(self, parent):
        self.ros2_setup()
        return "Subscribed to /kr3r540_sim/joint_state successfully."

    @uamethod
    async def shutdown_joint_state_subscriber(self, parent):
        """Cleanup method to stop ROS and threads."""
        if self.executor:
            self.executor.shutdown()
        if self.subscriber_node:
            self.subscriber_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "ROS 2 joint state subscriber shutdown successfully."