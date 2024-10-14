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
    
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_kr3r540_description = get_package_share_directory("kr3r540_description")
    pkg_kr3r540_bringup = get_package_share_directory("kr3r540_bringup")


    model_args=DeclareLaunchArgument(name="model",default_value=os.path.join(
        pkg_kr3r540_description, "urdf", "kr3r540.urdf.xacro"
    ),
    description="Abousolute path to the robot model file"
    )
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(pkg_kr3r540_description).parent.resolve())
        ],
    )

    robot_description = ParameterValue(
        Command(
            ["xacro ",LaunchConfiguration("model")]
        ),
        value_type=str,
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=PathJoinSubstitution([pkg_kr3r540_bringup, "rviz", "kr3r540.rviz"]),
        description="Path to the RViz config file."
    )

    params = {"robot_description" : robot_description,"use_sim_time": True}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[params],
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

    ros_distro = os.environ ['ROS_DISTRO']
    if ros_distro == "humble" :
        physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin" 

    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[
            ("gz_args",["-v 4 -r empty.sdf",physics_engine])
        ]
    )

    gazebo_bridge=Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )


    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=[
        "-topic", "robot_description",
        "-name","kr3r540",
    ]
    )

    return LaunchDescription([
        model_args,
        gazebo_resource_path,
        declare_sim_time,
        declare_rviz_config_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        ignition_launch,
        gazebo_bridge,
        spawn_entity,
    ])
