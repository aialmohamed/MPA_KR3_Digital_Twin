"""
Module : OPCUA_utils
Class: opcua_paths
Description: This class is used to get the paths of the configuration files and shell scripts.
Author: Ahmed Ibrahim Almohamed
Date: 10.11.2024
"""
#!/usr/bin/env python3
import sys 

from pathlib import Path

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

class opcua_paths():
    def __init__(self):
        self.root = current_dir.parent
        self.config = self.root / "config"
        self.opcua_config = self.config / "opcua_server_config.yaml"
        self.ros2_config = self.config / "ros2_configuration.yaml"
        self.roslibs = "opt/ros/humble/local/lib/python3.8/dist-packages"
        self.shell_path = self.root / "OPCUA_shell_scripts"
        self.shell_launch = self.shell_path / "System_launch.sh"
        self.shell_kill = self.shell_path / "System_kill.sh"
    
    def get_root(self):
        return self.root
    def get_config(self):
        return self.config
    def get_opcua_config(self):
        return self.opcua_config
    def get_roslibs(self):
        return self.roslibs        
    def get_ros2_config(self):
        return self.ros2_config
    def get_shell_path(self):
        return self.shell_path
    def get_shell_launch(self):
        return self.shell_launch
    def get_shell_kill(self):
        return self.shell_kill
    

