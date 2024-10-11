import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,SetEnvironmentVariable,ExecuteProcess,RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessExit
from pprint import pprint

import xacro

def generate_launch_description():
    urdf_path = LaunchConfiguration("urdf_path")
    world_path = LaunchConfiguration("world_path")
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_kr3r540_description = get_package_share_directory("kr3r540_description")
    pkg_kr3r540_bringup = get_package_share_directory("kr3r540_bringup")

    

    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [pkg_kr3r540_description, "urdf", "kr3r540.urdf"]
        ),
        description="Path to the URDF file of the robot."
    )

    declare_world_path = DeclareLaunchArgument(
        "world_path",
        default_value=PathJoinSubstitution([pkg_kr3r540_description, "world", "kr3r540_world.sdf"]),
        description="Path to the Ignition world file."
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=PathJoinSubstitution([pkg_kr3r540_bringup, "rviz", "kr3r540.rviz"]),
        description="Path to the RViz config file."
    )



    SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([get_package_share_directory('kr3r540_description'),"meshes"]))

    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}],
        arguments=[urdf_path],
        output='screen',
    )


    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True}],
        arguments=['-d', rviz_config_path ,'use_sim_time','true'],
        output="screen",
    )


    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments={
            'gz_args': world_path,
            'on_exit_shutdown': 'true'
            }.items(),
    )


    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{"use_sim_time": True}],
    arguments=[
        '-file', urdf_path,
        '-name', 'kr3r540',
        '-x', '0', '-y', '0', '-z', '0'
    ],
    output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )
    spawn_entity_reg = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        )

    
    load_joint_state_broadcaster_reg = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        )

    return LaunchDescription([
        declare_urdf_path,
        declare_world_path,
        declare_rviz_config_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        ignition_launch,
        spawn_entity_reg,
        load_joint_state_broadcaster_reg,
        spawn_entity,
    ])
