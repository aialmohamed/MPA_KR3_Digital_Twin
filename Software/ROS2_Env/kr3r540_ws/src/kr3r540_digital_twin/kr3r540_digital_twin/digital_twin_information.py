#!/usr/bin/env python3
"""
File: digital_twin_information.py
Author: Ahmed Ibrahim Almohamed
Email: ibraah03@thu.de
Date: 21.10.2024
Description: A node to sync data of the digital twin from the simulated robot to the real robot.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DigitalTwinInformation(Node):
    def __init__(self):
        super().__init__("digital_twin_information")
        
        # Subscribe to the digital twin's joint trajectory
        self.real_joint_state_sub = self.create_subscription(
            JointTrajectory, "/kr3r540_digital_twin/digital_twin_joint_trajectory", self.real_joint_state_callback, 10
        )
        
        # Publishers for real robot's arm and gripper controllers
        self.real_arm_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/arm_controller/joint_trajectory", 10
        )
        self.real_gripper_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/gripper_controller/joint_trajectory", 10
        )

        #self.get_logger().info("Digital shadow node started: syncing digital twin to real robot.")

    def real_joint_state_callback(self, msg):
        # Separate arm and gripper joints based on naming convention
        arm_joints = [name for name in msg.joint_names if "joint" in name]
        gripper_joints = [name for name in msg.joint_names if "flange_finger" in name]

        # Process and send arm trajectory
        if arm_joints:
            arm_trajectory_msg = JointTrajectory()
            arm_trajectory_msg.joint_names = arm_joints
            if msg.points:
                arm_point = JointTrajectoryPoint()
                arm_point.positions = [msg.points[0].positions[msg.joint_names.index(name)] for name in arm_joints]
                arm_point.time_from_start = rclpy.time.Duration(seconds=1).to_msg()
                arm_trajectory_msg.points.append(arm_point)
            self.real_arm_command_pub.publish(arm_trajectory_msg)
            #self.get_logger().info("Published arm trajectory to real robot.")

        # Process and send gripper trajectory
        if gripper_joints:
            gripper_trajectory_msg = JointTrajectory()
            gripper_trajectory_msg.joint_names = gripper_joints
            if msg.points:
                gripper_point = JointTrajectoryPoint()
                gripper_point.positions = [msg.points[0].positions[msg.joint_names.index(name)] for name in gripper_joints]
                gripper_point.time_from_start = rclpy.time.Duration(seconds=1).to_msg()
                gripper_trajectory_msg.points.append(gripper_point)
            self.real_gripper_command_pub.publish(gripper_trajectory_msg)
            #self.get_logger().info("Published gripper trajectory to real robot.")

def main(args=None):
    rclpy.init(args=args)
    digital_twin_data = DigitalTwinInformation()
    rclpy.spin(digital_twin_data)
    digital_twin_data.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
