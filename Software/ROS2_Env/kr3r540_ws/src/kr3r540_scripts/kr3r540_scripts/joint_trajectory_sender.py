#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import signal
import sys
import math

class JointTrajectorySender(Node):
    def __init__(self):
        super().__init__('joint_trajectory_sender')

        # Publishers for arm and gripper controllers
        self.arm_publisher = self.create_publisher(JointTrajectory, '/kr3r540_real/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/kr3r540_real/gripper_controller/joint_trajectory', 10)

        # Joint names for arm and gripper
        self.arm_joint_names = [f'joint_{i+1}' for i in range(6)]
        self.gripper_joint_names = ['flange_finger_1', 'flange_finger_2']

        # Initialize trajectories
        self.arm_trajectory = JointTrajectory()
        self.arm_trajectory.joint_names = self.arm_joint_names

        self.gripper_trajectory = JointTrajectory()
        self.gripper_trajectory.joint_names = self.gripper_joint_names

        # Arm joint limits in degrees
        self.joint_limits = {
            'joint_1': (-170, 170),
            'joint_2': (-170, 50),
            'joint_3': (-110, 155),
            'joint_4': (-175, 175),
            'joint_5': (-120, 120),
            'joint_6': (-350, 350),
            'flange_finger_1': (0, 1),  # Gripper range
            'flange_finger_2': (0, 1)   # Gripper range
        }

        signal.signal(signal.SIGINT, self.signal_handler)

    def send_trajectory(self):
        while rclpy.ok():
            try:
                # Input for arm joints
                arm_input = input(f"Geben Sie die Positionen in Grad für {', '.join(self.arm_joint_names)} ein (getrennt durch Kommas): ")
                positions_deg = [float(x) for x in arm_input.split(',')]
                if len(positions_deg) != 6:
                    raise ValueError("Es müssen genau 6 Werte eingegeben werden.")

                # Input for gripper joints
                gripper_input = input("Geben Sie die Positionen für die Greiferfinger ein (getrennt durch Komma, Werte zwischen 0 und 1): ")
                gripper_positions = [float(x) for x in gripper_input.split(',')]
                if len(gripper_positions) != 2:
                    raise ValueError("Es müssen genau 2 Werte für den Greifer eingegeben werden.")

                # Validate arm joint limits
                for i, pos in enumerate(positions_deg):
                    joint_name = self.arm_joint_names[i]
                    lower_limit, upper_limit = self.joint_limits[joint_name]
                    if not (lower_limit <= pos <= upper_limit):
                        raise ValueError(f"Position für {joint_name} muss zwischen {lower_limit}° und {upper_limit}° liegen.")

                # Validate gripper joint limits
                for i, pos in enumerate(gripper_positions):
                    joint_name = self.gripper_joint_names[i]
                    lower_limit, upper_limit = self.joint_limits[joint_name]
                    if not (lower_limit <= pos <= upper_limit):
                        raise ValueError(f"Position für {joint_name} muss zwischen {lower_limit} und {upper_limit} liegen.")

                # Convert degrees to radians for arm positions
                arm_positions_rad = [math.radians(pos) for pos in positions_deg]

                # Arm trajectory point
                arm_point = JointTrajectoryPoint()
                arm_point.positions = arm_positions_rad
                arm_point.time_from_start = rclpy.time.Duration(seconds=2.0).to_msg()

                # Gripper trajectory point
                gripper_point = JointTrajectoryPoint()
                gripper_point.positions = gripper_positions
                gripper_point.time_from_start = rclpy.time.Duration(seconds=2.0).to_msg()

                # Publish arm and gripper trajectories
                self.arm_trajectory.points = [arm_point]
                self.arm_publisher.publish(self.arm_trajectory)

                self.gripper_trajectory.points = [gripper_point]
                self.gripper_publisher.publish(self.gripper_trajectory)

                # Log info
                self.get_logger().info(f"Gesendete Arm-Positionen (in Grad): {positions_deg}")
                self.get_logger().info(f"Gesendete Greifer-Positionen: {gripper_positions}")

            except ValueError as e:
                self.get_logger().error(str(e))
                print("Bitte geben Sie die Positionen erneut korrekt ein.")
            except KeyboardInterrupt:
                print("Beenden mit Ctrl+C erkannt. Beende jetzt den Node...")
                break

    def signal_handler(self, sig, frame):
        print("Beenden mit Ctrl+C erkannt. Beende jetzt den Node...")
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectorySender()

    try:
        node.send_trajectory()
    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("Node wurde sauber beendet.")

if __name__ == '__main__':
    main()
