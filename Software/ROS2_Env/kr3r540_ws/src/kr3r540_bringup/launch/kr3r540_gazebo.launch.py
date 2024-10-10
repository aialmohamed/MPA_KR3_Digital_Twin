import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    LaunchConfiguration,
    FindExecutable,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configurations
    urdf_path = LaunchConfiguration("urdf_path")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    world_path = LaunchConfiguration("world_path")


    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("kr3r540_description"),
                "urdf",
                "kr3r540.urdf.xacro",
            ]
        ),
        description="Path to the URDF file of the robot.",
    )

    declare_yaml_config_path = DeclareLaunchArgument(
        "yaml_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("kr3r540_bringup"),
                "sim_config",
                "kr3r540_sim_controllers.yaml",
            ]
        ),
        description="Path to the YAML config file for controllers.",
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("kr3r540_bringup"),
                "rviz",
                "kr3r540_urdf_config.rviz",
            ]
        ),
        description="Path to the RViz config file.",
    )

    declare_world_path = DeclareLaunchArgument(
        "world_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("kr3r540_bringup"), "world", "kr3r540_world.world"]
        ),
        description="Path to the Gazebo world file.",
    )

    # Generate robot_description parameter by calling xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path,
    ])

    robot_description = {'robot_description': robot_description_content}

    # Include Gazebo launch file with the specified world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": world_path, "verbose": "true"}.items(),
    )

    print("Starting robot_state_publisher_node ...")
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    print("robot_state_publisher_node is started ")
    
    
    # Spawn Robot
    print("Spawning robot ...")
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "kr3r540",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
        ],
        output="screen",
    )
    print("Robot is spawned ")

    # Spawn Controllers
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kr3r540_sim_joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kr3r540_sim_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # RViz Node (optional)
    print("Starting RViz ...")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )
    print("RViz is started ")

    return LaunchDescription(
        [
            declare_urdf_path,
            declare_yaml_config_path,
            declare_rviz_config_path,
            declare_world_path,
            robot_state_publisher_node,
            gazebo_launch,
            spawn_robot_node,
            spawn_joint_state_broadcaster,
            spawn_joint_trajectory_controller,
        ]
    )
