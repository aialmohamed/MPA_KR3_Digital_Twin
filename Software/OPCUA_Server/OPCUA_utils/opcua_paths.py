#!/usr/bin/env python3
import sys 

from pathlib import Path
# Import the OPCUS_SERVER Folder (parent of the current directory)

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

class opcua_paths():
    def __init__(self):
        self.root = current_dir.parent
        self.config = self.root / "config"
        self.opcua_config = self.config / "opcua_server_config.yaml"
        self.ros2_config = self.config / "ros2_configuration.yaml"
        self.roslibs = "opt/ros/humble/local/lib/python3.8/dist-packages"
    
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

