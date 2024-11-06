import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from asyncua import ua
import sys 
from pathlib import Path
from rclpy.executors import MultiThreadedExecutor


current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_messages.JointsStateToOpcua import joints_state_to_opcua

class ros2_simulation_joint_state_subscriber(Node):
    def __init__(self, server_variables):
        super().__init__('joint_state_simulation_subscriber')
        self.server_variables = server_variables  # OPC UA variables to update

        # Create the ROS 2 subscription for JointState messages
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # Replace with your actual ROS 2 topic
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    async def listener_callback(self, msg):
        # Convert JointState to OPC UA-compatible format
        js_to_opcua = joints_state_to_opcua(msg)
        joint_data = js_to_opcua.joint_state_to_opcua()

        # Update OPC UA variables for each joint
        for joint_name, data in joint_data.items():
            if joint_name in self.server_variables:
                await self.server_variables[joint_name]["position"].write_value(data["position"])
                await self.server_variables[joint_name]["velocity"].write_value(data["velocity"])
                await self.server_variables[joint_name]["effort"].write_value(data["effort"])
        self.get_logger().info(f"Updated OPC UA server with joint state data.")