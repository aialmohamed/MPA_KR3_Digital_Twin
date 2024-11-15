#!/usr/bin/env python3
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # get the kr3r540_bringup :
    pkg_kr3r540_bringup = get_package_share_directory("kr3r540_bringup")

    # launching Real robot :
    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [pkg_kr3r540_bringup, "launch", "kr3r540_real.launch.py"]
                )
            ]
        ),
    )
    # launching real robot controllers :

    real_robot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [pkg_kr3r540_bringup, "launch", "kr3r540_real_controller.launch.py"]
                )
            ]
        ),
    )

    # launching gazebo simulation :
    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [pkg_kr3r540_bringup, "launch", "kr3r540_ign.launch.py"]
                )
            ]
        ),
    )

    # launching gazebo controllers :
    sim_robot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [pkg_kr3r540_bringup, "launch", "kr3r540_ign_controller.launch.py"]
                )
            ]
        ),
    )

    # launching the digital twin : 
    digital_twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [pkg_kr3r540_bringup, "launch", "digital_twin.launch.py"]
                )
            ]
        ),
    )

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
    
    return LaunchDescription(
        [
            sim_robot,
            sim_robot_controller,
            kinematics_server,
            real_robot,
            real_robot_controller,
            digital_twin_information_launcher,

        ]
    )
