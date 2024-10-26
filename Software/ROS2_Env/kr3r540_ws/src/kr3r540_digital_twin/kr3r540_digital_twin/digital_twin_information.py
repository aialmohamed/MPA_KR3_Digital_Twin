#!/usr/bin/env python3
"""
File: digital_twin_information.py
Author: Ahmed Ibrahim Almohamed
Email: ibraah03@thu.de
Date: 21.10.2024
Description: A node to sync data of the digital twin from the simulated robot to the real robot using an action client.
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DigitalTwinInformation(Node):
    def __init__(self):
        super().__init__("digital_twin_information")
        
        self.real_joint_state_sub = self.create_subscription(
            JointTrajectory, "/kr3r540_digital_twin/digital_twin_joint_trajectory", self.real_joint_state_callback, 10
        )
        self.sim_trajectory_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/arm_controller/joint_trajectory", 10
        )
        self.get_logger().info("Digital shadow node started: syncing real to sim")

    def real_joint_state_callback(self, msg):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = msg.joint_names

        if msg.points:
            point = msg.points[0]
            new_point = JointTrajectoryPoint()
            new_point.positions = point.positions
            new_point.time_from_start = rclpy.time.Duration(seconds=0.05).to_msg()
            trajectory_msg.points.append(new_point)

        self.get_logger().info(f"Publishing trajectory with {len(trajectory_msg.points)} points.")
        self.sim_trajectory_command_pub.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    digital_twin_data = DigitalTwinInformation()
    rclpy.spin(digital_twin_data)
    digital_twin_data.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
