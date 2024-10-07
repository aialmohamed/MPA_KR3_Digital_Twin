import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Use LaunchConfigurations for flexibility
    urdf_path = LaunchConfiguration("urdf_path")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    yaml_config_path = LaunchConfiguration("yaml_config_path")
    world_path = LaunchConfiguration("world_path")

    # Declare LaunchArguments to allow paths to be set from the command line
    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("kr3r540_description"), "urdf", "kr3r540.urdf.xacro"]
        ),
        description="Path to the URDF file of the robot."
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("kr3r540_bringup"), "rviz", "kr3r540_urdf_config.rviz"]
        ),
        description="Path to the RViz config file."
    )

    declare_yaml_config_path = DeclareLaunchArgument(
        "yaml_config_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("kr3r540_bringup"), "sim_config", "kr3r540_sim_controllers.yaml"]
        ),
        description="Path to the YAML config file for controllers."
    )

    declare_world_path = DeclareLaunchArgument(
        "world_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("kr3r540_bringup"), "world", "kr3r540_world.world"]
        ),
        description="Path to the Gazebo world file."
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="kr3r540_sim",
        parameters=[{"robot_description": Command([FindExecutable(name="xacro"), " ", urdf_path])}],
        remappings=[("/robot_description", "/kr3r540_sim/robot_description")]
    )

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world_path,
            "verbose": "true"
        }.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace="kr3r540_sim",
        arguments=[
            "-topic", "/kr3r540_sim/robot_description",
            "-entity", "kr3r540",
            "-x", "0", "-y", "0", "-z", "0",
            "-R", "0", "-P", "0", "-Y", "0"
        ],
        output="screen",
    )

    # Optional: Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="kr3r540_sim",
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="kr3r540_sim",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    # Controller Manager Node
    gazebo_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="kr3r540_sim",
        parameters=[{"robot_description": Command([FindExecutable(name="xacro"), " ", urdf_path])}, yaml_config_path],
        output="screen",
    )

    # Delay spawning of controllers to ensure correct order
    delayed_spawn_joint_state_broadcaster = TimerAction(
        period=5.0,  # Delay for 5 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["kr3r540_sim_joint_state_broadcaster", "--controller-manager", "/kr3r540_sim/controller_manager"],
                namespace="kr3r540_sim",
                output="screen",
            )
        ]
    )

    delayed_spawn_joint_trajectory_controller = TimerAction(
        period=10.0,  # Delay for 10 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["kr3r540_sim_joint_trajectory_controller", "--controller-manager", "/kr3r540_sim/controller_manager"],
                namespace="kr3r540_sim",
                output="screen",
            )
        ]
    )

    # Return the LaunchDescription with all nodes and declarations
    return LaunchDescription([
        declare_urdf_path,
        declare_rviz_config_path,
        declare_yaml_config_path,
        declare_world_path,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gazebo_ros2_control_node,
        delayed_spawn_joint_state_broadcaster,
        delayed_spawn_joint_trajectory_controller,
    ])
