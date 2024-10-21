import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class DigitalShadow(Node):
    def __init__(self):
        super().__init__('digital_shadow')

        self.real_joint_state_sub = self.create_subscription(
            JointState,
            '/kr3r540_real/joint_states', 
            self.real_joint_state_callback,
            10)
        self.sim_trajectory_command_pub = self.create_publisher(JointTrajectory, '/kr3r540_sim/arm_controller/joint_trajectory', 10)
        self.get_logger().info('Digital shadow node started: syncing real to sim')

    def real_joint_state_callback(self, msg):
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = msg.name  
        point = JointTrajectoryPoint()
        point.positions = msg.position 
        point.time_from_start = rclpy.time.Duration(seconds=0.05).to_msg() 
        trajectory_msg.points.append(point)
        self.sim_trajectory_command_pub.publish(trajectory_msg)
        self.get_logger().info('Published updated trajectory command to the simulation')

def main(args=None):
    rclpy.init(args=args)

    # Initialize the digital shadow node
    digital_shadow = DigitalShadow()

    # Spin to keep the node alive and listening
    rclpy.spin(digital_shadow)

    # Shutdown on exit
    digital_shadow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
