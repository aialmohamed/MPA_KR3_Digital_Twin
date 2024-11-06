#!/usr/bin/env python3
import yaml

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
            self.yaml_data = yaml.load(f, Loader=yaml.FullLoader)

    def get_opcua_ip(self):
        if self.data != None:
            self.opcua_ip = self.yaml_data['ip']
        else:
            print("Error: No data loaded")
        return self.opcua_ip
    def get_opcua_port(self):
        if self.data != None:
            self.opcua_port = self.yaml_data['port']
        else:
            print("Error: No data loaded")
        return self.opcua_port
    def get_opcua_namespace(self):
        if self.data != None:
            self.opcua_namespace = self.yaml_data['namespace']
        else:
            print("Error: No data loaded")
        return self.opcua_namespace
    def get_opcua_name(self):
        if self.data != None:
            self.opcua_name = self.yaml_data['name']
        else:
            print("Error: No data loaded")
        return self.opcua_name
    def get_security_policy(self):
        if self.data != None:
            self.security_policy = self.yaml_data['security_policy']
        else:
            print("Error: No data loaded")
        return self.security_policy
    
