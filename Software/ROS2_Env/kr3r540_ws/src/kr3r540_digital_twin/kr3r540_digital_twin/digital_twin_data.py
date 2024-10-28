#!/usr/bin/env python3
"""
File: digital_twin_data.py
Author: Ahmed Ibrahim Almohamed
Email: ibraah03@thu.de
Date: 16.10.2024
Description: a node to pass data of the digital twin from the real robot to the simulated robot
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class DigitalTwinData(Node):
    def __init__(self):
        super().__init__("digital_twin_data")


        self.real_joint_state_sub = self.create_subscription(
            JointState, "/kr3r540_real/joint_states", self.real_joint_state_callback, 10
        )
        

        self.dt_state_pub = self.create_publisher(
            JointState, "/kr3r540_digital_twin/digital_twin_joint_state", 10
        )

        self.dt_state_sub = self.create_subscription(
            JointState, "/kr3r540_digital_twin/digital_twin_joint_state", self.dt_state_callback, 10
        )
        
        self.sim_trajectory_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_sim/arm_controller/joint_trajectory", 10
        )

        #self.get_logger().info("Digital shadow node started: syncing real to sim")

    def real_joint_state_callback(self, msg):

        self.dt_state_pub.publish(msg)
        #self.get_logger().info("Published joint state to digital twin joint state topic.")

    def dt_state_callback(self, msg):

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = msg.name

        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start = rclpy.time.Duration(seconds=0.05).to_msg()
        
        trajectory_msg.points.append(point)
        self.sim_trajectory_command_pub.publish(trajectory_msg)
        
        #self.get_logger().info("Published trajectory to simulated robot's arm controller.")

def main(args=None):
    rclpy.init(args=args)
    digital_twin_data = DigitalTwinData()
    rclpy.spin(digital_twin_data)
    digital_twin_data.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
