import sys
from pathlib import Path
from asyncua import uamethod, ua
import threading
import subprocess
import os
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

from OPCUA_utils.opcua_paths import opcua_paths
from OPCUA_config.ros2_configuration import ros2_configuration



class ros2_simulation_methods:
    def __init__(self):
        super().__init__()
        self.launch_process = None
        ros2_paths = opcua_paths()
        self.ros2_config = ros2_configuration(ros2_paths.get_ros2_config())
        self.ros2_config.loaddata()
        self.ros2_launch_file = ros2_paths.get_shell_launch()
        self.ros2_kill_file = ros2_paths.get_shell_kill()
        self.launch_thread = None
        self.kill_thread = None



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
            shell_script_path = self.ros2_launch_file 
            subprocess.run(["chmod", "+x", shell_script_path], check=True)
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
        