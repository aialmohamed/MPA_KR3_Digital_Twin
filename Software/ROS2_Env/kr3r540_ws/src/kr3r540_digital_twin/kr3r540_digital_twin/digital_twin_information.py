#!/usr/bin/env python3
"""
File: digital_twin_information.py
Author: Ahmed Ibrahim Almohamed
Email: ibraah03@thu.de
Date: 21.10.2024
Description: a node to pass data of the digital twin from the simulated robot to the real robot

"""


import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState 


class DigitalTwinInformation(Node):
    def __init__(self):
        super().__init__('digital_twin_information')

        self.real_joint_state_sub = self.create_subscription(
            JointState,
            '/kr3r540_real/joint_states', 
            self.real_joint_state_callback,
            10)
        self.sim_trajectory_command_pub = self.create_publisher(JointState, '/kr3r540_sim/joint_states', 10)
        self.get_logger().info('Digital shadow node started: syncing real to sim')

    def real_joint_state_callback(self, msg):
    
        trajectory_msg = JointState()
        trajectory_msg.joint_names = msg.name  
        #point = JointTrajectoryPoint()
        #point.positions = msg.position 
        #point.time_from_start = rclpy.time.Duration(seconds=0.05).to_msg() 
        #trajectory_msg.points.append(point)
        trajectory_msg.position = msg.position
        self.sim_trajectory_command_pub.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    digital_twin_data = DigitalTwinInformation()
    rclpy.spin(digital_twin_data)
    digital_twin_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
