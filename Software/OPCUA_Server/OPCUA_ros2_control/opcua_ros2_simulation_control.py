from abc import ABC, abstractmethod
import sys
import rclpy
from sensor_msgs.msg import JointState
from pathlib import Path
from asyncua import uamethod
from rclpy.executors import MultiThreadedExecutor
import threading
import subprocess
import os
import asyncio
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
import signal

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
        self.launch_thread = None

    @uamethod
    async def launch_ros2_simulation(self,parent):
        """Starts the ROS 2 launch file in a separate thread."""
        if self.launch_thread is not None and self.launch_thread.is_alive():
            return "ROS 2 launch is already running."

        # Start the launch process in a separate thread
        self.launch_thread = threading.Thread(target=self._start_ros2_launch)
        self.launch_thread.start()

    def _start_ros2_launch(self):
        try:
            ros_setup = "~/.bashrc"
            command = f". {ros_setup} && ros2 launch {' '.join(self.launch_file_path)}"
            self.launch_process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            self.monitor_process()

            print(f"ROS 2 launch PID: {self.launch_process.pid}")


            return self.launch_process.stdout.read().decode()
        except Exception as e:
            print(f"Failed to start ROS 2 launch file: {e}")
            return f"Error: {str(e)}"
        
    def monitor_process(self):
        """Monitors the output of the ROS 2 launch process."""
        if self.launch_process:
            while True:
                line = self.launch_process.stdout.readline()
                if line == b'':  # Check if process has ended
                    break
                print(f"ROS 2 Output: {line.decode().strip()}")

            self.launch_process.wait()
            print("ROS 2 process finished.")
            self.launch_process = None

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
        """Stops the ROS 2 launch file and all its associated processes."""
        if self.launch_process:
            try:
                # Terminate the entire process group
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait()  # Wait for the process to terminate
                print("ROS 2 launch stopped.")
                self.launch_process = None  # Clear the reference to the process
                return "ROS 2 launch stopped successfully."
            except Exception as e:
                print(f"Failed to stop ROS 2 launch file: {e}")
                return f"Error stopping ROS 2 launch: {str(e)}"
        else:
            return "No ROS 2 launch is running."

if __name__ == "__main__":
    control = ros2_simulation_control()
    control.start_ros2_subscriber()
    #print("Successfully initialized ros2_simulation_control.")