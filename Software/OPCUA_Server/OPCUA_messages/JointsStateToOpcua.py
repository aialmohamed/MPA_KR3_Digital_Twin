#!/usr/bin/env python3
import sys
import rclpy
from sensor_msgs.msg import JointState
from pathlib import Path

# Import the OPCUS_SERVER Folder (parent of the current directory)

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_utils.opcua_paths import opcua_paths


class joints_state_to_opcua:
    def __init__(self, joint_state_: JointState):
        if not isinstance(joint_state_, JointState):
            raise TypeError("joint_state_ must be of type sensor_msgs.msg.JointState")

        self.joint_state = joint_state_
        self.joint_names = self.joint_state.name
        self.joint_positions = list(self.joint_state.position)
        self.joint_velocities = list(self.joint_state.velocity)
        self.joint_efforts = list(self.joint_state.effort)

    def get_joint_names(self):
        return self.joint_names

    def get_joint_positions(self):
        return self.joint_positions

    def get_joint_velocities(self):
        return self.joint_velocities

    def get_joint_efforts(self):
        return self.joint_efforts
    def joint_state_to_opcua(self):
        joint_data = {}
        for name in self.joint_names:
            joint_data[name] = {
                "position": self.joint_positions[self.joint_names.index(name)],
                "velocity": self.joint_velocities[self.joint_names.index(name)],
                "effort": self.joint_efforts[self.joint_names.index(name)],
            }
        return joint_data
