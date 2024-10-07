import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/kr3r540_sim/kr3r540_sim_joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        # Joint names
        goal_msg.trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        
        # First point - current position
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=0)
        goal_msg.trajectory.points.append(point)
        
        # Second point - desired position
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # Adjust these values as needed
        point.time_from_start = Duration(sec=2)
        goal_msg.trajectory.points.append(point)
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = RobotMover()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
