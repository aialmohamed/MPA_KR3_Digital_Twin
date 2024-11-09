#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

        #launching action server : 
    kinematics_server = Node(
        package="kr3r540_kinematics_action_server",
        executable="kr3r540_kinematics_action_server_node",
        name="kr3r540_kinematics_action_server",
        namespace="kr3r540_sim",
        output="screen",
    )
    digital_twin_information_launcher=Node(
    package="kr3r540_digital_twin",
    namespace="kr3r540_digital_twin",
    executable="digital_twin_information",
    )

    return LaunchDescription([
        kinematics_server,
        digital_twin_information_launcher,
    ])