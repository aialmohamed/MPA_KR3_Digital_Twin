import yaml
from asyncua import ua
from pathlib import Path

class ros2_configuration:

    def __init__(self,Path):
        self.yaml_path = Path
        self.yaml_data = None
        self.ros2_launch_pkg = None
        self.ros2_launch_files_category = None
        self.ros2_simulation_launch_files = None
    
    def loaddata(self):
        with open(self.yaml_path,'r') as f:
            self.data = yaml.load(f, Loader=yaml.FullLoader)
        if 'ros2_configuration' in self.data:
            self.yaml_data = self.data['ros2_configuration']
        else:
            print("Error: 'ros2_configuration' key not found in YAML data")
    def get_ros2_launch_pkg(self):
        if self.yaml_data is None:
            raise ValueError("Error: No data loaded")
        self.ros2_launch_pkg = self.yaml_data['ros2_pkgs']
        return self.ros2_launch_pkg
    

    def get_ros2_launch_files_category(self):
        if self.yaml_data is None:
            raise ValueError("Error: No data loaded")
        self.ros2_launch_files_category = self.yaml_data['ros2_launchers']
        return self.ros2_launch_files_category
    
    def get_ros2_simulation_launch_files(self):
        if self.yaml_data is None:
            raise ValueError("Error: No data loaded")
        catagories = self.get_ros2_launch_files_category()
        self.ros2_simulation_launch_files = catagories['simulation']
        return self.ros2_simulation_launch_files
    def get_ros2_real_launch_files(self):
        if self.yaml_data is None:
            raise ValueError("Error: No data loaded")
        catagories = self.get_ros2_launch_files_category()
        self.ros2_real_launch_files = catagories['real']
        return self.ros2_real_launch_files
    def get_ros2_system_launch(self):
        if self.yaml_data is None:
            raise ValueError("Error: No data loaded")
        catagories = self.get_ros2_launch_files_category()
        self.ros2_system_launch = catagories['system']
        return self.ros2_system_launch
    def get_ros2_digital_twin_launch(self):
        if self.yaml_data is None:
            raise ValueError("Error: No data loaded")
        catagories = self.get_ros2_launch_files_category()
        self.ros2_digital_twin_launch = catagories['digital_twin']
        return self.ros2_digital_twin_launch