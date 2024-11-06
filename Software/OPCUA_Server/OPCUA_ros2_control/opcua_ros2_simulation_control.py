from abc import ABC, abstractmethod
import sys
import rclpy
from sensor_msgs.msg import JointState
from pathlib import Path
from asyncua import uamethod
from rclpy.executors import MultiThreadedExecutor
import threading
import subprocess

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

from OPCUA_ros2_control.opcua_ros2_control import ros2_control
from OPCUA_ros2_control.ros2_simulation_joint_state import ros2_simulation_joint_state_subscriber


class ros2_simulation_control(ros2_control):
    def __init__(self,opcua_server):
        super().__init__()
        self.opcua_server = opcua_server
        self.ros_node = None
        self.server_variables = {}
        self.executor_thread = None
        self.executor = None
        self.launch_process = None
        self.launch_file_path = ["kr3r540_bringup", "kr3r540_ign.launch.py"]

    @uamethod
    async def launch_ros2_simulation(self,parent):
        try:
            # Run the hardcoded ROS 2 launch file path
            ros_setup = "/opt/ros/humble/setup.bash"
            command = f"source {ros_setup} && ros2 launch {' '.join(self.launch_file_path)}"
            self.launch_process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            print(f"Started ROS 2 launch file: {' '.join(self.launch_file_path)}")
            return "ROS 2 launch started successfully."
        except Exception as e:
            print(f"Failed to start ROS 2 launch file: {e}")
            return f"Error: {str(e)}"
            

    def ros2_execution(self):
        # Initialize rclpy in this thread
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        
        # Add the ROS 2 node to the executor
        self.executor.add_node(self.ros_node)
        
        # Start spinning the executor to handle ROS 2 callbacks
        self.executor.spin()
        
        # Shutdown rclpy when done
        rclpy.shutdown()

    @uamethod
    async def start_ros2_subscriber(self,parent,joint_names):
        
        joint_folder = await self.opcua_server.nodes.objects.add_folder("ns=2;s=Joints", "Joints")
        for joint_name in joint_names:
            self.server_variables[joint_name] = {
                "position": await joint_folder.add_variable("ns=2;s={}Position".format(joint_name), "{}Position".format(joint_name), 0.0),
                "velocity": await joint_folder.add_variable("ns=2;s={}Velocity".format(joint_name), "{}Velocity".format(joint_name), 0.0),
                "effort": await joint_folder.add_variable("ns=2;s={}Effort".format(joint_name), "{}Effort".format(joint_name), 0.0)
            }
        self.ros_node = ros2_simulation_joint_state_subscriber(self.server_variables)
        self.executor_thread = threading.Thread(target=self.ros2_execution)
        self.executor_thread.start()
        print("ROS 2 subscriber started.")

    @uamethod
    async def stop_ros2_subscriber(self):
        if self.ros_node:
            self.ros_node.destroy_node()
            rclpy.shutdown()
            self.executor_thread.join()
            print("ROS 2 Subscriber Node stopped.")
    
    @uamethod
    async def stop_ros2_launch(self, parent):
        """Stops the ROS 2 launch file."""
        if self.launch_process:
            self.launch_process.terminate()
            self.launch_process.wait()  # Wait for the process to terminate
            print("ROS 2 launch stopped.")
            return "ROS 2 launch stopped successfully."
        else:
            return "No ROS 2 launch is running."

if __name__ == "__main__":
    control = ros2_simulation_control()
    control.start_ros2_subscriber()
    #print("Successfully initialized ros2_simulation_control.")