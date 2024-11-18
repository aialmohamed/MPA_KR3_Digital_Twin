#!/usr/bin/env python3
import sys 

from pathlib import Path
# Import the OPCUA_SERVER Folder (parent of the current directory)

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

from OPCUA_config.opcua_configuration import opcua_configuration
from OPCUA_utils.opcua_paths import opcua_paths
import asyncio
import logging
from asyncua import Server, ua,uamethod
from OPCUA_ros2_control.opcua_ros2_simulation_methods import ros2_simulation_methods
from OPCUA_ros2_control.opcua_ros2_digital_twin_methods import ros2_digital_twin_methods
from OPCUA_ros2_control.opcua_ros2_real_pose_methods import ros2_real_pose_methods
from OPCUA_ros2_control.opcua_ros2_goal_method import ros2_goals_methods
from OPCUA_ros2_control.opcua_ros2_simulation_joint_state_methods import ros2_simulation_joint_state_methods
from OPCUA_ros2_control.opcua_ros2_sim_pose_methods import ros2_sim_pose_methods
from kr3r540_msgs.msg import CartesianPose
class opcua_ros2_server():
    def __init__(self,config_path):


        self.opcua_server = Server()
        

        self.config_path = config_path
        self.opcua_config = opcua_configuration(self.config_path)

        self.simulation_control = None
        self.digital_twin_control = None
        self.ros2_pose_control = None
        self.ros2_goal_control = None
        self.ros2_joint_state_control = None
        self.ros2_sim_pose_control = None

        self.opcua_config.load_configuration()
        self.ip = self.opcua_config.get_opcua_ip()
        self.port = self.opcua_config.get_opcua_port()
        self.namespace = self.opcua_config.get_opcua_namespace()
        self.name = self.opcua_config.get_opcua_name()
        self.security_policy = self.opcua_config.get_security_policy()

        self.joint_state_dict = {}
        self.cartPose_real = None
        self.cartPose_sim = None

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
        await self.create_cartesian_pose_real()
        await self.create_cartesian_pose_sim()
        self.simulation_control = ros2_simulation_methods()
        self.digital_twin_control = ros2_digital_twin_methods()
        self.ros2_pose_control = ros2_real_pose_methods(self.cartPose_real)
        self.ros2_goal_control = ros2_goals_methods()
        self.ros2_sim_pose_control = ros2_sim_pose_methods(self.cartPose_sim)
        self.ros2_joint_state_control = ros2_simulation_joint_state_methods(self.joint_state_dict)
        await self.add_ros2_simulation_methods()
        await self.add_digital_twin_methods()
        await self.add_joint_states_methods()
        await self.add_cartesian_pose_methods()
        await self.add_goal_methods()

        self.opcua_server.set_security_policy([self.security_policy])

        
    async def add_digital_twin_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=DigitalTwinMethods", "DigitalTwinMethods")
        await obj.add_method(ua.NodeId("LaunchDigitalTwin",self.opcua_idx), ua.QualifiedName("LaunchDigitalTwin",self.opcua_idx), self.digital_twin_control.launch_ros2_digital_twin, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("ShutdownDigitalTwin",self.opcua_idx), ua.QualifiedName("ShutdownDigitalTwin",self.opcua_idx), self.digital_twin_control.shutdown_ros2_digital_twin, [], [ua.VariantType.String])
    
    async def add_ros2_simulation_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=ControlMethods", "SimulationControlMethods")
        await obj.add_method(ua.NodeId("LaunchRos2Simulation",self.opcua_idx), ua.QualifiedName("LaunchRos2Simulation",self.opcua_idx), self.simulation_control.launch_ros2_simulation, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("KillRos2Simulation",self.opcua_idx), ua.QualifiedName("KillRos2Simulation",self.opcua_idx), self.simulation_control.kill_ros2_simulation, [], [ua.VariantType.String])
        

    
    async def add_joint_states_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=JointStateMethods", "JointStateMethods")
        await obj.add_method(ua.NodeId("SubscribeToJointState",self.opcua_idx), ua.QualifiedName("SubscribeToJointState",self.opcua_idx), self.ros2_joint_state_control.subscribe_to_joint_state, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("UnsubscribeToJointState",self.opcua_idx), ua.QualifiedName("UnsubscribeToJointState",self.opcua_idx), self.ros2_joint_state_control.shutdown_joint_state_subscriber, [], [ua.VariantType.String])

    async def add_cartesian_pose_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=CartesianPoseMethods", "CartesianPoseMethods")
        await obj.add_method(ua.NodeId("GetRealCartesianPose",self.opcua_idx), ua.QualifiedName("GetRealCartesianPose",self.opcua_idx), self.ros2_pose_control.get_robot_cartesian_pose, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("ShutdownRealCartesianPose",self.opcua_idx), ua.QualifiedName("ShutdownRealCartesianPose",self.opcua_idx), self.ros2_pose_control.shutdown_robot_cartesian_pose, [], [ua.VariantType.String])
        
        await obj.add_method(ua.NodeId("GetSimCartesianPose",self.opcua_idx), ua.QualifiedName("GetSimCartesianPose",self.opcua_idx), self.ros2_sim_pose_control.get_robot_cartesian_pose, [], [ua.VariantType.String])
        await obj.add_method(ua.NodeId("ShutdownSimCartesianPose",self.opcua_idx), ua.QualifiedName("ShutdownSimCartesianPose",self.opcua_idx), self.ros2_sim_pose_control.shutdown_robot_cartesian_pose, [], [ua.VariantType.String])
    async def add_goal_methods(self):
        obj = await self.opcua_server.nodes.objects.add_folder("ns=2;s=GoalMethods", "GoalMethods")
        input_args =self.create_send_goal_input_args()
        await obj.add_method(ua.NodeId("SendGoal",self.opcua_idx),ua.QualifiedName("SendGoal",self.opcua_idx),self.ros2_goal_control.send_ros_goal,
                             input_args,
                             [ua.VariantType.String])
    
    def create_send_goal_input_args(self):          

        inargx = ua.Argument()
        inargx.Name = "x"
        inargx.DataType = ua.NodeId(ua.ObjectIds.Float)
        inargx.ValueRank = -1
        inargx.ArrayDimensions = []
        inargx.Description = ua.LocalizedText("x in meters")


        inargy = ua.Argument()
        inargy.Name = "y"
        inargy.DataType = ua.NodeId(ua.ObjectIds.Float)
        inargy.ValueRank = -1
        inargy.ArrayDimensions = []
        inargy.Description = ua.LocalizedText("y in meters")

        inargz = ua.Argument()
        inargz.Name = "z"
        inargz.DataType = ua.NodeId(ua.ObjectIds.Float)
        inargz.ValueRank = -1
        inargz.ArrayDimensions = []
        inargz.Description = ua.LocalizedText("z in meters")

        inargroll = ua.Argument()
        inargroll.Name = "roll"
        inargroll.DataType = ua.NodeId(ua.ObjectIds.Float)
        inargroll.ValueRank = -1
        inargroll.ArrayDimensions = []
        inargroll.Description = ua.LocalizedText("roll in degrees")

        inargpitch = ua.Argument()
        inargpitch.Name = "pitch"
        inargpitch.DataType = ua.NodeId(ua.ObjectIds.Float)
        inargpitch.ValueRank = -1
        inargpitch.ArrayDimensions = []
        inargpitch.Description = ua.LocalizedText("pitch in degrees")

        inargyaw = ua.Argument()
        inargyaw.Name = "yaw"
        inargyaw.DataType = ua.NodeId(ua.ObjectIds.Float)
        inargyaw.ValueRank = -1
        inargyaw.ArrayDimensions = []
        inargyaw.Description = ua.LocalizedText("yaw in degrees")

        inarggripper = ua.Argument()
        inarggripper.Name = "gripper_state"
        inarggripper.DataType = ua.NodeId(ua.ObjectIds.Int32)
        inarggripper.ValueRank = -1
        inarggripper.ArrayDimensions = []
        inarggripper.Description = ua.LocalizedText("gripper state (0 for closed, 1 for open)")

        return [inargx, inargy, inargz, inargroll, inargpitch, inargyaw, inarggripper]
        
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
    async def create_cartesian_pose_real(self):
        objects_node = self.opcua_server.nodes.objects
        cartesian_pose_folder = await objects_node.add_folder(self.opcua_idx, "CartesianPoseReal")
        cartesian_obj = await cartesian_pose_folder.add_object(self.opcua_idx, "CartesianPose")
        

        self.x_opcua = await cartesian_obj.add_variable(self.opcua_idx, "x", 0.0)
        self.y_opcua = await cartesian_obj.add_variable(self.opcua_idx, "y", 0.0)
        self.z_opcua = await cartesian_obj.add_variable(self.opcua_idx, "z", 0.0)
        self.roll_opcua = await cartesian_obj.add_variable(self.opcua_idx, "roll", 0.0)
        self.pitch_opcua = await cartesian_obj.add_variable(self.opcua_idx, "pitch", 0.0)
        self.yaw_opcua = await cartesian_obj.add_variable(self.opcua_idx, "yaw", 0.0)
        self.gripper_opcua = await cartesian_obj.add_variable(self.opcua_idx, "gripper",False, ua.VariantType.Boolean)
        
        await self.x_opcua.set_writable()
        await self.y_opcua.set_writable()
        await self.z_opcua.set_writable()
        await self.roll_opcua.set_writable()
        await self.pitch_opcua.set_writable()
        await self.yaw_opcua.set_writable()
        await self.gripper_opcua.set_writable()
        self.cartPose_real = {
            "x": self.x_opcua,
            "y": self.y_opcua,
            "z": self.z_opcua,
            "roll": self.roll_opcua,
            "pitch": self.pitch_opcua,
            "yaw": self.yaw_opcua,
            "gripper": self.gripper_opcua,
        }
    async def create_cartesian_pose_sim(self):
        objects_node = self.opcua_server.nodes.objects
        cartesian_pose_folder = await objects_node.add_folder(self.opcua_idx, "CartesianPoseSimulation")
        cartesian_obj = await cartesian_pose_folder.add_object(self.opcua_idx, "CartesianPose")
        

        self.x_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "x", 0.0)
        self.y_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "y", 0.0)
        self.z_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "z", 0.0)
        self.roll_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "roll", 0.0)
        self.pitch_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "pitch", 0.0)
        self.yaw_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "yaw", 0.0)
        self.gripper_opcua_sim = await cartesian_obj.add_variable(self.opcua_idx, "gripper",False, ua.VariantType.Boolean)
        
        await self.x_opcua_sim.set_writable()
        await self.y_opcua_sim.set_writable()
        await self.z_opcua_sim.set_writable()
        await self.roll_opcua_sim.set_writable()
        await self.pitch_opcua_sim.set_writable()
        await self.yaw_opcua_sim.set_writable()
        await self.gripper_opcua_sim.set_writable()
        self.cartPose_sim = {
            "x": self.x_opcua_sim,
            "y": self.y_opcua_sim,
            "z": self.z_opcua_sim,
            "roll": self.roll_opcua_sim,
            "pitch": self.pitch_opcua_sim,
            "yaw": self.yaw_opcua_sim,
            "gripper": self.gripper_opcua_sim,
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


