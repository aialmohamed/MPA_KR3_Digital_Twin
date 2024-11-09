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
        

        self.config_path = config_path
        # load configuration
        self.opcua_config = opcua_configuration(self.config_path)
        self.simulation_control = None
        self.opcua_config.load_configuration()
        self.ip = self.opcua_config.get_opcua_ip()
        self.port = self.opcua_config.get_opcua_port()
        self.namespace = self.opcua_config.get_opcua_namespace()
        self.name = self.opcua_config.get_opcua_name()
        self.security_policy = self.opcua_config.get_security_policy()
        self.joint_state_dict = {}
        print(self.security_policy)

        # Opcua endpoints : 
        self.endpoints = f"opc.tcp://{self.ip}:{self.port}"
        self.uri = f'http://{self.namespace}'
        self.opcua_idx = None

    async def init_opcua_server(self):
        await self.opcua_server.init()
        self.opcua_server.set_endpoint(self.endpoints)
        self.opcua_idx = await self.opcua_server.register_namespace(self.uri)
        await self.create_joint_states_ros2_simulation()
        self.simulation_control = ros2_simulation_control(self.opcua_server,self.joint_state_dict)     
        await self.add_ros2_simulation_methods()
        await self.add_digital_twin_methods()
        self.opcua_server.set_security_policy([self.security_policy])

        
    async def add_digital_twin_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=DigitalTwinMethods", "DigitalTwinMethods")
        await obj.add_method(ua.NodeId("LaunchDigitalTwin",self.opcua_idx), ua.QualifiedName("LaunchDigitalTwin",self.opcua_idx), self.simulation_control.launch_ros2_digital_twin, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("ShutdownDigitalTwin",self.opcua_idx), ua.QualifiedName("ShutdownDigitalTwin",self.opcua_idx), self.simulation_control.shutdown_ros2_digital_twin, [], [ua.VariantType.String])
    async def add_ros2_simulation_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=ControlMethods", "SimulationControlMethods")
        await obj.add_method(ua.NodeId("LaunchRos2Simulation",self.opcua_idx), ua.QualifiedName("LaunchRos2Simulation",self.opcua_idx), self.simulation_control.launch_ros2_simulation, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("SubscribeToJointState",self.opcua_idx), ua.QualifiedName("SubscribeToJointState",self.opcua_idx), self.simulation_control.subscribe_to_joint_state, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("UnsubscribeToJointState",self.opcua_idx), ua.QualifiedName("UnsubscribeToJointState",self.opcua_idx), self.simulation_control.shutdown_joint_state_subscriber, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("SendGoal",self.opcua_idx),ua.QualifiedName("SendGoal",self.opcua_idx),self.simulation_control.send_ros_goal,
                             [ua.VariantType.Float, ua.VariantType.Float, ua.VariantType.Float, ua.VariantType.Float, ua.VariantType.Float, ua.VariantType.Int32],
                             [ua.VariantType.String])
    async def create_joint_states_ros2_simulation(self):
        objects_node = self.opcua_server.nodes.objects
        joint_state_folder = await objects_node.add_folder(self.opcua_idx, "JointStateSimulation")

        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "flange_finger_1", "flange_finger_2"]
        self.joint_state_dict = {}
        for joint_name in joint_names:
            joint_object = await joint_state_folder.add_object(self.opcua_idx, joint_name)
            position_variable = await joint_object.add_variable(self.opcua_idx, f"{joint_name}_Position", 0.0)
            velocity_variable = await joint_object.add_variable(self.opcua_idx, f"{joint_name}_Velocity", 0.0)
            effort_variable = await joint_object.add_variable(self.opcua_idx, f"{joint_name}_Effort", 0.0)
            await position_variable.set_writable()
            await velocity_variable.set_writable()
            await effort_variable.set_writable()
            self.joint_state_dict[joint_name] = {
                "Position": position_variable,
                "Velocity": velocity_variable,
                "Effort": effort_variable
            }


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


