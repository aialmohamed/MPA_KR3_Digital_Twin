#!/usr/bin/env python3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from asyncua import ua
import sys
from pathlib import Path

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_messages.JointsStateToOpcua import joints_state_to_opcua


class ros2_simulation_joint_state_subscriber(Node):
    def __init__(self, server_variable_dict):
        super().__init__(
            "simulation_joint_state_subscriber", namespace="opcua"
        )
        self.joint_state_data = None
        self.server_variable_dict = server_variable_dict
        self.sim_joint_state_sub = self.create_subscription(
            JointState, "/kr3r540_sim/joint_states", self.listener_callback, 10
        )

    async def listener_callback(self, msg):
        data = joints_state_to_opcua(msg)
        self.joint_state_data = data.joint_state_to_opcua()

        for joint_name, joint_data in self.joint_state_data.items():
            joint_object = self.server_variable_dict.get(joint_name)
            if joint_object:

                position = joint_data.get("position", 0.0)
                velocity = joint_data.get("velocity", 0.0)
                effort = joint_data.get("effort", 0.0)

                await joint_object["Position"].write_value(position)
                await joint_object["Velocity"].write_value(velocity)
                await joint_object["Effort"].write_value(effort)
                self.get_logger().info(
                    f"Updated OPC UA joint state for {joint_name}: Position={position}, Velocity={velocity}, Effort={effort}"
                )
