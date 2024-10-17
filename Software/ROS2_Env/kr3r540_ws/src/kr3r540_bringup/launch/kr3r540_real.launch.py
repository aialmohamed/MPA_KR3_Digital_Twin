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
            ),
            " is_sim:=False"
            ]
        ),
        value_type=str,
    )


    params = {"robot_description" : robot_description,"use_sim_time": False}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        namespace="kr3r540_real",
        executable="robot_state_publisher",
        parameters=[params],
        output='screen',
    )

    controller_manager_node = Node(
        package="controller_manager",
        namespace="kr3r540_real",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": False},
             os.path.join(
                 get_package_share_directory("kr3r540_bringup"),
                 "real_config",
                 "kr3r540_real_controllers.yaml"
             )
        ],
        remappings=[('~/robot_description','/kr3r540_real/robot_description'),],
        output='screen',
    )

    
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
    ])