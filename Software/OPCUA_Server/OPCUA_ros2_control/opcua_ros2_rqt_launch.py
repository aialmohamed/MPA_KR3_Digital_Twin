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



class ros2_rqt_methods:
    def __init__(self):
        super().__init__()
        self.launch_process = None
        ros2_paths = opcua_paths()
        self.ros2_config = ros2_configuration(ros2_paths.get_ros2_config())
        self.ros2_config.loaddata()
        self.rqt_path = ros2_paths.get_rqt_launch()
        self.launch_thread = None
        self.kill_thread = None



    @uamethod
    async def launch_rqt(self, parent):
        
        if self.launch_thread is not None and self.launch_thread.is_alive():
            return "Rqt  launch is already running."
        try:
            self.launch_thread = threading.Thread(target=self._start_ros2_launch)
            self.launch_thread.start()
            return "Rqt launch started successfully."
        except Exception as e:
            print(f"Failed to start ROS 2 launch file: {e}")
            return f"Error starting ROS 2 launch: {str(e)}"

    def _start_ros2_launch(self):
        try:
            shell_script_path = self.rqt_path
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