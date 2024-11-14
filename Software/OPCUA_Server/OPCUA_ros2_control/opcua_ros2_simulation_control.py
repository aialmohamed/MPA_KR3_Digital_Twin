import sys
import rclpy
from sensor_msgs.msg import JointState
from pathlib import Path
from asyncua import uamethod, ua
from rclpy.executors import SingleThreadedExecutor
import threading
import subprocess
import os
import asyncio
import psutil

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_ros2_control.ros2_simulation_joint_state_node import (
    ros2_simulation_joint_state_subscriber,
)
from OPCUA_utils.opcua_paths import opcua_paths
from OPCUA_config.ros2_configuration import ros2_configuration
from OPCUA_ros2_control.ros2_digital_twin import ros2_digital_twin
import time

import signal

class ros2_simulation_control:
    def __init__(self, opcua_server, joint_state_server_variable):
        super().__init__()
        self.opcua_server = opcua_server
        self.joint_state_server_variable = joint_state_server_variable
        ## ros2 simulation launch
        self.launch_process = None
        ros2_paths = opcua_paths()
        self.ros2_config = ros2_configuration(ros2_paths.get_ros2_config())
        self.ros2_config.loaddata()
        self.ros2_launch_file = ros2_paths.get_shell_launch()
        self.ros2_kill_file = ros2_paths.get_shell_kill()




        ## joint state subscriber
        self.launch_thread = None
        self.kill_thread = None
        self.subscriber_node = None
        self.executor = None
        self.joint_state_data = None

        ## send goal
        self.command = None

        # digital twin 
        self.digital_twin_node = None
        self.digital_twin_executor = None
        self.digital_twin_thread = None


    @uamethod
    async def launch_ros2_simulation(self, parent):
        """Starts the ROS 2 launch file in a separate thread."""
        if self.launch_thread is not None and self.launch_thread.is_alive():
            return "ROS 2 launch is already running."
        try:
            self.launch_thread = threading.Thread(target=self._start_ros2_launch)
            self.launch_thread.start()
            return "ROS 2 launch started successfully."
        except Exception as e:
            print(f"Failed to start ROS 2 launch file: {e}")
            return f"Error starting ROS 2 launch: {str(e)}"

    def _start_ros2_launch(self):
        try:
            # Path to the shell script
            shell_script_path = self.ros2_launch_file 

            # Ensure the script is executable

            subprocess.run(["chmod", "+x", shell_script_path], check=True)

            # Run the shell script
            self.launch_process = subprocess.Popen(
                ["bash", "-c", shell_script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            
            return "Shell script executed successfully."
        except Exception as e:
            print(f"Failed to run the shell script: {e}")
            return f"Error: {str(e)}"
    @uamethod
    def kill_ros2_simulation(self, parent):
        if self.kill_thread is not None and self.kill_thread.is_alive():
            return "ROS 2 launch is already running."
        try:
            self.kill_thread = threading.Thread(target=self._kill_ros2_launch)
            self.kill_thread.start()
            return "ROS 2 launch is killed successfully."
        except Exception as e:
            print(f"Failed to kill ROS 2 launch file: {e}")
            return f"Error killing ROS 2 launch: {str(e)}"

    def _kill_ros2_launch(self):
        try:
            # Path to the shell script
            shell_script_path = self.ros2_kill_file

            # Ensure the script is executable

            subprocess.run(["chmod", "+x", shell_script_path], check=True)

            # Run the shell script
            self.launch_process = subprocess.Popen(
                ["bash", "-c", shell_script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            
            return "Shell script executed successfully."
        except Exception as e:
            print(f"Failed to run the shell script: {e}")
            return f"Error: {str(e)}"       
        


    def ros2_setup(self):
        if not rclpy.ok():
            rclpy.init()
        self.subscriber_node = ros2_simulation_joint_state_subscriber(
            self.joint_state_server_variable
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.subscriber_node)
        self.ros_thread = threading.Thread(target=self.spin_with_delay)
        self.ros_thread.start()

    def spin_with_delay(self):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.5)

    @uamethod
    async def subscribe_to_joint_state(self, parent):
        self.ros2_setup()
        return "Subscribed to /kr3r540_sim/joint_state successfully."

    @uamethod
    async def shutdown_joint_state_subscriber(self, parent):
        """Cleanup method to stop ROS and threads."""
        if self.executor:
            self.executor.shutdown()
        if self.subscriber_node:
            self.subscriber_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "ROS 2 joint state subscriber shutdown successfully."

    @uamethod
    async def send_ros_goal(
        self,
        parent,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        finger1: int,
    ):
        self.command = (
            f"ros2 action send_goal /kr3r540_kinematics/kr3r540_kinematics kr3r540_msgs/action/Kr3r540Kinemactis "
            f'"cartesian_goal: [{x},{y},{z},{roll},{pitch},{finger1},{finger1}]" -f'
        )

        goal_thread = threading.Thread(target=self.execute_command)
        goal_thread.start()

        return [ua.Variant("Goal command executed", ua.VariantType.String)]
    
    def execute_command(self):

        os.system(self.command)

    ## digital Twin launcher
    def _start_digital_twin(self):
        if not rclpy.ok():
            rclpy.init()
        
        self.digital_twin_node = ros2_digital_twin()
        self.digital_twin_executor = SingleThreadedExecutor()
        self.digital_twin_executor.add_node(self.digital_twin_node)
        self.digital_twin_executor_thread = threading.Thread(target=self.digital_twin_executor.spin)
        self.digital_twin_executor_thread.start()


    @uamethod
    async def launch_ros2_digital_twin(self, parent):
        self._start_digital_twin()
        return "ROS 2 Digital Twin started successfully."


    @uamethod
    async def shutdown_ros2_digital_twin(self, parent):
        if self.digital_twin_executor:
            self.digital_twin_executor.shutdown()
        if self.digital_twin_node:
            self.digital_twin_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "ROS 2 Digital Twin shutdown successfully."
            

        


