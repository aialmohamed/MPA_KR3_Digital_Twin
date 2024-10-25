#!/usr/bin/env python3
"""
File: digital_twin_information.py
Author: Ahmed Ibrahim Almohamed
Email: ibraah03@thu.de
Date: 21.10.2024
Description: A node to sync data of the digital twin from the simulated robot to the real robot using an action client.
"""
from array import array
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kr3r540_msgs.action import Kr3r540Kinemactis  # Ensure this matches your action message type
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DigitalTwinInformation(Node):
    def __init__(self):
        super().__init__('digital_twin_information')

        # Action client for the kinematics action server
        self.action_client = ActionClient(self, Kr3r540Kinemactis, '/kr3r540_kinematics')

        # Publisher to send joint trajectories to the real robot
        self.sim_trajectory_command_pub = self.create_publisher(JointTrajectory, '/kr3r540_real/arm_controller/joint_trajectory', 10)

        # Send a goal to the action server
        self.send_goal()

    def send_goal(self):
        # Define the goal (example Cartesian coordinates)
        goal_msg = Kr3r540Kinemactis.Goal()
        goal_msg.cartesian_goal = [0.2, -0.2, 0.5]  # Customize with desired goal

        # Wait for the action server to be ready
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # Send the goal
        self.get_logger().info('Sending goal to action server...')
        self._goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the action server')
            return
        self.get_logger().info('Goal accepted by the action server')
        
        # Register the result callback
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the action server
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.joints_cartesian_feedback}')

        # Publish feedback as trajectory commands to the real robot
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']  # Customize as needed
        point = JointTrajectoryPoint()
        point.positions = array('d', feedback.joints_cartesian_feedback)
        point.time_from_start = rclpy.time.Duration(seconds=0.05).to_msg()
        trajectory_msg.points.append(point)
        self.sim_trajectory_command_pub.publish(trajectory_msg)

    def result_callback(self, future):
        # Handle the final result from the action server
        result = future.result().result
        self.get_logger().info(f'Received result: {result.joints_result}')


def main(args=None):
    rclpy.init(args=args)
    digital_twin_data = DigitalTwinInformation()
    rclpy.spin(digital_twin_data)
    digital_twin_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
