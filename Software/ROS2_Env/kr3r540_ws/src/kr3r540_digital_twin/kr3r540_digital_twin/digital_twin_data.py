#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.time import Duration

class DigitalTwinData(Node):
    def __init__(self):
        super().__init__("digital_twin_data")
        

        self.sim_joint_state_sub = self.create_subscription(
            JointState, "/kr3r540_sim/joint_states", self.real_joint_state_callback, 10
        )
        

        self.real_trajectory_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/arm_controller/joint_trajectory", 10
        )
        self.real_gripper_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/gripper_controller/joint_trajectory", 10
        )

        self.last_joint_state_msg = None
        self.prev_joint_positions = []
        self.prev_gripper_positions = []
        self.movement_stopped = False
        self.gripper_movement_stopped = False

    def real_joint_state_callback(self, msg):

        if not all(pos == 0.0 for pos in msg.position):
            self.last_joint_state_msg = msg


            current_positions = [pos for name, pos in zip(msg.name, msg.position) if "joint" in name]
            if self.prev_joint_positions:
                self.movement_stopped = all(
                    abs(current - prev) < 1e-4 
                    for current, prev in zip(current_positions, self.prev_joint_positions)
                )
            self.prev_joint_positions = current_positions


            current_gripper_positions = [pos for name, pos in zip(msg.name, msg.position) if "flange_finger" in name]
            if self.prev_gripper_positions:
                self.gripper_movement_stopped = all(
                    abs(current - prev) < 1e-6 
                    for current, prev in zip(current_gripper_positions, self.prev_gripper_positions)
                )
            self.prev_gripper_positions = current_gripper_positions


            if not self.movement_stopped:
                self.publish_to_robot_arm()
            if not self.gripper_movement_stopped:
                self.publish_to_robot_gripper()

    def publish_to_robot_arm(self):
        if self.last_joint_state_msg is None:
            return

        arm_names = [name for name in self.last_joint_state_msg.name if "joint" in name]
        if arm_names:
            arm_trajectory_msg = JointTrajectory()
            arm_trajectory_msg.joint_names = arm_names
            arm_point = JointTrajectoryPoint()
            for name in arm_names:
                arm_point.positions.append(
                    self.last_joint_state_msg.position[self.last_joint_state_msg.name.index(name)]
                )
            arm_point.time_from_start = Duration(seconds=0.1).to_msg()
            arm_trajectory_msg.points.append(arm_point)
            self.real_trajectory_command_pub.publish(arm_trajectory_msg)
    
    def publish_to_robot_gripper(self):
        if self.last_joint_state_msg is None:
            return
        
        gripper_names = [name for name in self.last_joint_state_msg.name if "flange_finger" in name]
        if gripper_names:
            gripper_trajectory_msg = JointTrajectory()
            gripper_trajectory_msg.joint_names = gripper_names
            gripper_point = JointTrajectoryPoint()
            for name in gripper_names:
                gripper_point.positions.append(
                    self.last_joint_state_msg.position[self.last_joint_state_msg.name.index(name)]
                )
            gripper_point.time_from_start = Duration(seconds=0.04).to_msg()
            gripper_trajectory_msg.points.append(gripper_point)
            self.real_gripper_command_pub.publish(gripper_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    digital_twin_data = DigitalTwinData()
    rclpy.spin(digital_twin_data)
    digital_twin_data.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
