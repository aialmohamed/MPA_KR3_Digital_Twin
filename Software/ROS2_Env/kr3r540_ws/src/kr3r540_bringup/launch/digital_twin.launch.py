from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    digital_twin_data_launcher=Node(
        package="kr3r540_digital_twin",
        namespace="kr3r540_digital_twin",
        executable="digital_twin_data",
    )

    digital_twin_information_launcher=Node(
    package="kr3r540_digital_twin",
    namespace="kr3r540_digital_twin",
    executable="digital_twin_information",
    )

    return LaunchDescription([
        digital_twin_data_launcher,
        digital_twin_information_launcher
    ])