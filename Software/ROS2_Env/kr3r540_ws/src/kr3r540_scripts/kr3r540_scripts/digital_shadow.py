import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class DigitalShadow(Node):
    def __init__(self):
        super().__init__('digital_shadow')

        # Subscribe to the real robot's joint states
        self.real_joint_state_sub = self.create_subscription(
            JointState,
            '/kr3r540_real/joint_states',  # Topic from the real robot
            self.real_joint_state_callback,
            10)

        # Publisher to the simulation's joint trajectory command topic
        self.sim_trajectory_command_pub = self.create_publisher(JointTrajectory, '/kr3r540_sim/arm_controller/joint_trajectory', 10)

        self.get_logger().info('Digital shadow node started: syncing real to sim')

    def real_joint_state_callback(self, msg):
        # Log the received data from the real robot (optional)
        #self.get_logger().info(msg)

        # Create a JointTrajectory message for the simulated robot
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()  # Use current time for sim
        trajectory_msg.joint_names = msg.name  # Joint names from the real robot

        # Create a JointTrajectoryPoint message to hold the positions
        point = JointTrajectoryPoint()
        point.positions = msg.position  # Use the positions from the real robot
        point.time_from_start.sec = 1  # Set duration for the trajectory (1 second in this case)

        # Add the point to the trajectory
        trajectory_msg.points.append(point)

        # Publish to the simulation's trajectory command topic
        self.sim_trajectory_command_pub.publish(trajectory_msg)
        #self.get_logger().info('Published trajectory command to the simulation')


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
