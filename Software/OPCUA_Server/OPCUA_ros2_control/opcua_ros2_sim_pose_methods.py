import sys
import rclpy
from pathlib import Path
from asyncua import uamethod
from rclpy.executors import SingleThreadedExecutor
import threading
from kr3r540_msgs.msg import CartesianPose

current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))
from OPCUA_ros2_nodes.ros2_sim_robot_pose import sim_robot_cartesian_pose_subscriber


class ros2_sim_pose_methods:
    def __init__ (self, cartPose_sim):
        # cartesian pose
        self.cartPose_sim = cartPose_sim
        self.cartesian_pose_node = None
        self.cartesian_pose_executor = None
        self.cartesian_pose_thread = None

   
    
    @uamethod
    async def get_robot_cartesian_pose(self, parent):
        self._start_robot_pose()
        return "Robot Cartesian Pose started successfully."
    
    @uamethod
    async def shutdown_robot_cartesian_pose(self, parent):
        if self.cartesian_pose_executor:
            self.cartesian_pose_executor.shutdown()
        if self.cartesian_pose_node:
            self.cartesian_pose_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "Robot Cartesian Pose shutdown successfully."

    def _start_robot_pose(self):
        if not rclpy.ok():
            rclpy.init()
        self.cartesian_pose_node = sim_robot_cartesian_pose_subscriber(self.cartPose_sim)
        self.cartesian_pose_executor = SingleThreadedExecutor()
        self.cartesian_pose_executor.add_node(self.cartesian_pose_node)
        self.cartesian_pose_thread = threading.Thread(target=self.cartesian_pose_executor.spin)
        self.cartesian_pose_thread.start()