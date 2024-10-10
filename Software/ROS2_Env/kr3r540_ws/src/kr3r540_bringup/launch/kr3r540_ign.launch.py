import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable

def generate_launch_description():
    # Declare launch arguments for paths
    urdf_path = LaunchConfiguration("urdf_path")
    world_path = LaunchConfiguration("world_path")
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    # Get the package directories for required files
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_kr3r540_description = get_package_share_directory("kr3r540_description")
    pkg_kr3r540_bringup = get_package_share_directory("kr3r540_bringup")

    # Define the paths to the URDF, RViz, and world files using the package directories
    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [pkg_kr3r540_description, "urdf", "kr3r540.urdf.xacro"]
        ),
        description="Path to the URDF file of the robot."
    )

    declare_world_path = DeclareLaunchArgument(
        "world_path",
        default_value=PathJoinSubstitution([pkg_kr3r540_description, "world", "default_world.sdf"]),
        description="Path to the Ignition world file."
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=PathJoinSubstitution([pkg_kr3r540_bringup, "rviz", "kr3r540.rviz"]),
        description="Path to the RViz config file."
    )

    # Generate robot_description parameter by calling xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path,
    ])

    robot_description = {'robot_description': robot_description_content}

    SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([get_package_share_directory('kr3r540_description'),"meshes"]))


    # Node to publish the robot's URDF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    # Joint State Publisher node for publishing joint states
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    # RViz Node to visualize the robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        output="screen",
    )

    # Include launch file to start Ignition Gazebo with the specified world
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_kr3r540_description, 'world', 'default_world.sdf'
            ]),
            'on_exit_shutdown': 'true'
            }.items(),
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-file', urdf_path,
        '-name', 'kr3r540',
        '-x', '0', '-y', '0', '-z', '0'
    ],
    output='screen'
    )
    

    # Return the launch description with all the nodes and configurations
    return LaunchDescription([
        declare_urdf_path,
        declare_world_path,
        declare_rviz_config_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        ignition_launch,
        spawn_entity,
    ])
