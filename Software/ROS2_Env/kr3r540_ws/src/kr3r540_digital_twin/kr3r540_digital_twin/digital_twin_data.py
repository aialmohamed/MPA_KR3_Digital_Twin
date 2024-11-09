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
        self.dt_state_sub = self.create_subscription(
            JointState,
            "/kr3r540_digital_twin/digital_twin_joint_state",
            self.dt_state_callback,
            10,
        )


        self.dt_state_pub = self.create_publisher(
            JointState, "/kr3r540_digital_twin/digital_twin_joint_state", 10
        )
        self.real_trajectory_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/arm_controller/joint_trajectory", 10
        )
        self.real_gripper_command_pub = self.create_publisher(
            JointTrajectory, "/kr3r540_real/gripper_controller/joint_trajectory", 10
        )


        self.last_joint_state_msg = None
        self.timer = self.create_timer(1, self.process_joint_state) 

    def real_joint_state_callback(self, msg):

        self.dt_state_pub.publish(msg)

    def dt_state_callback(self, msg):

        if not all(pos == 0.0 for pos in msg.position) or self.last_joint_state_msg:
            self.last_joint_state_msg = msg

    def process_joint_state(self):

        if self.last_joint_state_msg:
            msg = self.last_joint_state_msg
            

            arm_names = [name for name in msg.name if "joint" in name]
            gripper_names = [name for name in msg.name if "flange_finger" in name]


            if arm_names:
                arm_trajectory_msg = JointTrajectory()
                arm_trajectory_msg.joint_names = arm_names
                arm_point = JointTrajectoryPoint()
                for name in arm_names:
                    arm_point.positions.append(msg.position[msg.name.index(name)])
                arm_point.time_from_start = Duration(seconds=0.2).to_msg()
                arm_trajectory_msg.points.append(arm_point)
                self.real_trajectory_command_pub.publish(arm_trajectory_msg)

            # Process and send gripper trajectory
            if gripper_names:
                gripper_trajectory_msg = JointTrajectory()
                gripper_trajectory_msg.joint_names = gripper_names
                gripper_point = JointTrajectoryPoint()
                for name in gripper_names:
                    gripper_point.positions.append(msg.position[msg.name.index(name)])
                gripper_point.time_from_start = Duration(seconds=0.2).to_msg()
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
