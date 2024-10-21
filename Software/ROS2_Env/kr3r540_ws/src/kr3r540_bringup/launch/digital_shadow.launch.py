from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    digital_twin_launcher=Node(
        package="kr3r540_scripts",
        namespace="kr3r540_digital_twin",
        executable="digital_shadow",
    )

    return LaunchDescription([
        digital_twin_launcher,
    ])