import os
from launch import LaunchDescription
from pathlib import Path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,SetEnvironmentVariable,ExecuteProcess,RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():



    robot_description = ParameterValue(
        Command(
            ["xacro ",
            os.path.join(
                get_package_share_directory("kr3r540_description"),
                "urdf",
                "kr3r540.urdf.xacro"
            )
            ]
        ),
        value_type=str,
    )


    params = {"robot_description" : robot_description,"use_sim_time": True}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[params],
        output='screen',
    )


    joint_state_broadcaster_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    arm_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    

    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])