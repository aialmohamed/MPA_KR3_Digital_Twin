#!/usr/bin/env python3
import yaml
from asyncua import ua

class opcua_configuration:
    def __init__(self,Path):
        self.yaml_path = Path
        self.yaml_data = None
        self.opcua_ip = None
        self.opcua_port = None
        self.opcua_namespace = None
        self.opcua_name = None
        self.security_policy = None
    def load_configuration(self):
        with open(self.yaml_path,'r') as f:
            self.data = yaml.load(f, Loader=yaml.FullLoader)
        if 'opcua_server' in self.data:
            self.yaml_data = self.data['opcua_server']
        else:
            print("Error: 'opcua_server' key not found in YAML data")
            #TODO : refactor this with if and raise error
    def get_opcua_ip(self):
        if self.yaml_data is not None:
            self.opcua_ip = self.yaml_data['ip']
        else:
            print("Error: No data loaded")
        return self.opcua_ip
    def get_opcua_port(self):
        if self.yaml_data is not None:
            self.opcua_port = self.yaml_data['port']
        else:
            print("Error: No data loaded")
        return self.opcua_port
    def get_opcua_namespace(self):
        if self.yaml_data is not None:
            self.opcua_namespace = self.yaml_data['namespace']
        else:
            print("Error: No data loaded")
        return self.opcua_namespace
    def get_opcua_name(self):
        if self.yaml_data is not None:
            self.opcua_name = self.yaml_data['name']
        else:
            print("Error: No data loaded")
        return self.opcua_name
    def get_security_policy(self):
        if self.yaml_data  is not  None:
            security_policy_str = self.yaml_data['security_policy']
            try:
                self.security_policy = getattr(ua.SecurityPolicyType, security_policy_str.split('.')[-1])
            except AttributeError:
                print(f"Error: Security policy '{security_policy_str}' is not valid")
                self.security_policy = ua.SecurityPolicyType.NoSecurity
        else:
            print("Error: No data loaded")
        return self.security_policy
    
