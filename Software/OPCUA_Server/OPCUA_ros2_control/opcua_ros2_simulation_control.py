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
        pkg = self.ros2_config.get_ros2_launch_pkg()
        self.simulation_pkg = pkg[0]
        self.system_launch = self.ros2_config.get_ros2_system_launch()[0]
        self.digital_twin_launch = self.ros2_config.get_ros2_digital_twin_launch()[0]


        ## joint state subscriber
        self.launch_thread = None

        self.subscriber_node = None
        self.executor = None
        self.joint_state_data = None

        ## send goal
        self.command = None

        # digital twin 
        self.digital_twin_thread = None
        self.digital_twin_command = None
        self.digital_twin_process = None

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
            ros_setup = "~/.bashrc"
            command = f". {ros_setup}  && ros2 launch {self.simulation_pkg} {self.system_launch}"
            self.launch_process = subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            self.monitor_process()
            return self.launch_process.stdout.read().decode()
        except Exception as e:
            print(f"Failed to start ROS 2 launch file: {e}")
            return f"Error: {str(e)}"

    def monitor_process(self):
        """Monitors the output of the ROS 2 launch process."""
        if self.launch_process:
            while True:
                line = self.launch_process.stdout.readline()
                if line == b"":  # Check if process has ended
                    break
                print(f"ROS 2 Output: {line.decode().strip()}")

            self.launch_process.wait()
            print("ROS 2 process finished.")
            print(f"ROS 2 launch PID: {self.launch_process.pid}")
            self.launch_process = None

    def ros2_setup(self):
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
        rclpy.shutdown()

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
            f"ros2 action send_goal /kr3r540_sim/kr3r540_kinematics kr3r540_msgs/action/Kr3r540Kinemactis "
            f'"cartesian_goal: [{x},{y},{z},{roll},{pitch},{finger1},{finger1}]" -f'
        )

        goal_thread = threading.Thread(target=self.execute_command)
        goal_thread.start()

        return [ua.Variant("Goal command executed", ua.VariantType.String)]
    
    def execute_command(self):

        os.system(self.command)

    ## digital Twin launcher
    def _start_digital_twin(self):
        try:
            self.digital_twin_command = f". ~/.bashrc && ros2 launch {self.simulation_pkg} {self.digital_twin_launch}"
            self.digital_twin_process = subprocess.Popen(
                ["gnome-terminal", "--","bash", "-c", self.digital_twin_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                shell=False,
            )
            print(f"ROS 2 digital twin PID: {self.digital_twin_process.pid}")
            return f"ROS 2 digital twin PID: {self.digital_twin_process.pid}"
        except Exception as e:
            print(f"Failed to start ROS 2 digital twin: {e}")
            return f"Error: {str(e)}"

    
    @uamethod
    async def launch_ros2_digital_twin(self, parent):
        try:
            self.digital_twin_thread = threading.Thread(target=self._start_digital_twin)
            self.digital_twin_thread.start()
            return "ROS 2 digital twin started successfully."
        except Exception as e:
            print(f"Failed to start ROS 2 digital twin: {e}")
            return f"Error starting ROS 2 digital twin: {str(e)}"
        

    @uamethod
    async def shutdown_ros2_digital_twin(self, parent):
        """Cleanup method to stop ROS and threads."""
        try:
            print(f"pid: {self.digital_twin_process.pid}")
            if self.digital_twin_process:
                self.kill_process_and_childern()
                
                # Wait for the thread to finish
                self.digital_twin_thread.join()
                return "ROS 2 digital twin stopped successfully."
            else:
                return "No ROS 2 digital twin process to stop."
        except Exception as e:
            print(f"Failed to stop ROS 2 digital twin: {e}")
            return f"Error stopping ROS 2 digital twin: {str(e)}"
            
    def kill_process_and_childern(self):
        try:
            parent = psutil.Process(self.digital_twin_process.pid)
            children = parent.children(recursive=True)
            for child in children:
                print(f"Child process {child.name} {child.pid}  have parent : {parent.name} at {parent.pid}.")
                if "gnome-terminal" in child.name():
                    child.terminate()
                    print(f"Child process {child.name} {child.pid} terminated.")
                    break
        except psutil.NoSuchProcess:
            print("No such process")

        


