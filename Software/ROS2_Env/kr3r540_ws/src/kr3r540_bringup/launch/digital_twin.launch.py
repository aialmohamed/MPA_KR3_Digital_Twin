from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    digital_twin_launcher=Node(
        package="kr3r540_digital_twin",
        namespace="kr3r540_digital_twin",
        executable="digital_twin_data",
    )

    return LaunchDescription([
        digital_twin_launcher,
    ])