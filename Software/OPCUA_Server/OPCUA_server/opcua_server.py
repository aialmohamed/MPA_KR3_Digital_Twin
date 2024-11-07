#!/usr/bin/env python3
import sys 

from pathlib import Path
# Import the OPCUS_SERVER Folder (parent of the current directory)

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

from OPCUA_config.opcua_configuration import opcua_configuration
from OPCUA_utils.opcua_paths import opcua_paths
import asyncio
import logging
from asyncua import Server, ua,uamethod
from OPCUA_ros2_control.opcua_ros2_simulation_control import ros2_simulation_control

class opcua_ros2_server():
    def __init__(self,config_path):

        # Opcua Server :
        self.opcua_server = Server()
        self.simulation_control = ros2_simulation_control(self.opcua_server)

        self.config_path = config_path
        # load configuration
        self.opcua_config = opcua_configuration(self.config_path)

        self.opcua_config.load_configuration()
        self.ip = self.opcua_config.get_opcua_ip()
        self.port = self.opcua_config.get_opcua_port()
        self.namespace = self.opcua_config.get_opcua_namespace()
        self.name = self.opcua_config.get_opcua_name()
        self.security_policy = self.opcua_config.get_security_policy()
        print(self.security_policy)

        # Opcua endpoints : 
        self.endpoints = f"opc.tcp://{self.ip}:{self.port}"
        self.uri = f'http://{self.namespace}'
        self.opcua_idx = None

    async def init_opcua_server(self):
        await self.opcua_server.init()
        self.opcua_server.set_endpoint(self.endpoints)
                
        # Register start and stop methods for the ROS 2 subscriber
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=ControlMethods", "SimulationControlMethods")
        await obj.add_method("ns=2;s=StopROS2Simulation", "StopROS2Simulation", self.simulation_control.stop_ros2_launch, [], [ua.VariantType.String])
        await obj.add_method("ns=2;s=LaunchROS2Simulation", "LaunchROS2Simulation", self.simulation_control.launch_ros2_simulation, [], [ua.VariantType.String])
        await obj.add_method("ns=2;s=StartROS2Subscriber", "StartROS2Subscriber", self.simulation_control.start_ros2_subscriber, [ua.VariantType.String], [ua.VariantType.String])
        await obj.add_method("ns=2;s=StopROS2Subscriber", "StopROS2Subscriber", self.simulation_control.stop_ros2_subscriber, [], [ua.VariantType.String])
        self.opcua_server.set_security_policy([self.security_policy])
        self.opcua_idx = await self.opcua_server.register_namespace(self.uri)


    async def start_opcua_server(self):
        async with self.opcua_server:
            print("OPC UA Server running.")
            while True:
                await asyncio.sleep(0.1)
    async def stop_opcua_server(self):
        await self.opcua_server.stop()
        
        


async def main():
    paths = opcua_paths()
    configuration_file = paths.get_opcua_config()
    server = opcua_ros2_server(configuration_file)
    await server.init_opcua_server()
    try:
        await server.start_opcua_server()
    finally:
        await server.stop_opcua_server()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server interrupted by user. Shutting down...")


