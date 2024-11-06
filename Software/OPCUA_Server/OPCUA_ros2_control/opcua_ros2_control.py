from abc import ABC, abstractmethod
from asyncua import uamethod

class ros2_control(ABC):
    def __init__(self):
        pass
    
    @uamethod
    @abstractmethod
    async def start_ros2_subscriber(self,parent,joint_names):
        """ Abstract method to read the joints state from the robot """
        pass
    