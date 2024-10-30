from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace="kr3r540_real",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/kr3r540_real/controller_manager"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="kr3r540_real",
        arguments=["arm_controller", "-c", "/kr3r540_real/controller_manager"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="kr3r540_real",
        arguments=["gripper_controller", "-c", "/kr3r540_real/controller_manager"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
