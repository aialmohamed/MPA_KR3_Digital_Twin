from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,SetEnvironmentVariable,ExecuteProcess,RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    pkg_kr3r540_description = get_package_share_directory("kr3r540_description")
    pkg_kr3r540_bringup = get_package_share_directory("kr3r540_bringup")


    model_args=DeclareLaunchArgument(name="model",default_value=os.path.join(
        pkg_kr3r540_description, "urdf", "kr3r540.urdf.xacro"
    ),
    description="Abousolute path to the robot model file"
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', os.path.join(pkg_kr3r540_bringup, "rviz", "kr3r540.rviz")],
        output="screen",
    )

    robot_description = ParameterValue(
        Command(
            ["xacro ",LaunchConfiguration("model")]
        ),
        value_type=str,
    )
    joint_stat_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    params = {"robot_description" : robot_description}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[params],
        output='screen',
    )
    return LaunchDescription([
        model_args,
        rviz_node,
        joint_stat_publisher_gui_node,
        robot_state_publisher_node
    ])