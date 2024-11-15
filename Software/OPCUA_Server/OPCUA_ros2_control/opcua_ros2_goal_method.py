import sys
import rclpy
from sensor_msgs.msg import JointState
from pathlib import Path
from asyncua import uamethod, ua
from rclpy.executors import SingleThreadedExecutor
import threading
import os
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))


class ros2_goals_methods:
    def __init__(self):
        self.command = None
    
    @uamethod
    async def send_ros_goal(
        self,
        parent,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        finger1: int,
    ):
        self.command = (
            f"ros2 action send_goal /kr3r540_kinematics/kr3r540_kinematics kr3r540_msgs/action/Kr3r540Kinemactis "
            f'"cartesian_goal: [{x},{y},{z},{roll},{pitch},{yaw},{finger1},{finger1}]" -f'
        )

        goal_thread = threading.Thread(target=self.execute_command)
        goal_thread.start()

        return [ua.Variant("Goal command executed", ua.VariantType.String)]
    def execute_command(self):
        os.system(self.command)