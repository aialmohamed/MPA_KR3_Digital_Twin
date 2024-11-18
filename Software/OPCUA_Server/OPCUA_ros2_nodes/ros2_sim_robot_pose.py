#!/usr/bin/env python3
from rclpy.node import Node

from kr3r540_msgs.msg import CartesianPose
import sys
from pathlib import Path
from rclpy.time import Duration
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

class sim_robot_cartesian_pose_subscriber(Node):
    def __init__(self, cartPose_sim):
        self.cartPose_sim = cartPose_sim
        super().__init__("simulation_robot_cartesian_pose_subscriber", namespace="opcua")
        self._subscription = self.create_subscription(
            CartesianPose, '/opcua/sim_robot_cartesian_pose', self.cartesian_pose_callback, 10
        )
        self.get_logger().info("SimRobotCartesianPoseSubscriber node has been initialized.")
    async def cartesian_pose_callback(self, msg: CartesianPose):
        x = self.round_to_zero(msg.x)
        y = self.round_to_zero(msg.y)
        z = self.round_to_zero(msg.z)
        roll = self.round_to_zero(msg.roll)
        pitch = self.round_to_zero(msg.pitch)
        yaw = self.round_to_zero(msg.yaw)
        self.get_logger().info(f"Received Cartesian Pose: x={msg.x}, y={msg.y}, z={msg.z}, roll={msg.roll}, pitch={msg.pitch}, yaw={msg.yaw}, gripper={msg.gripper}")
        await self.cartPose_sim["x"].set_value(x)
        await self.cartPose_sim["y"].set_value(y)
        await self.cartPose_sim["z"].set_value(z)
        await self.cartPose_sim["roll"].set_value(roll)
        await self.cartPose_sim["pitch"].set_value(pitch)
        await self.cartPose_sim["yaw"].set_value(yaw)
        await self.cartPose_sim["gripper"].set_value(msg.gripper)
    
    def round_to_zero(self,value, threshold=1e-6):
        return 0.0 if abs(value) < threshold else value
    