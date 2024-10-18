#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import signal
import sys
import math  # Importiere die math-Bibliothek für die Umrechnung von Grad zu Radiant

class JointTrajectorySender(Node):
    def __init__(self):
        super().__init__('joint_trajectory_sender')

        # Publisher für den JointTrajectory-Befehl
        self.publisher = self.create_publisher(JointTrajectory, '/kr3r540_real/arm_controller/joint_trajectory', 10)

        # Liste der Gelenknamen (für 6 Gelenke)
        self.joint_names = [f'joint_{i+1}' for i in range(6)]

        # Initialisiere den JointTrajectory-Nachrichtentyp
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = self.joint_names

        # Gelenkgrenzen in Grad (hier hartcodiert, in deinem Fall vielleicht dynamisch)
        self.joint_limits = {
            'joint_1': (-170, 170),  # entspricht -2.967 bis 2.967 Radiant
            'joint_2': (-170, 50),   # entspricht -2.967 bis 0.872 Radiant
            'joint_3': (-110, 155),  # entspricht -1.919 bis 2.705 Radiant
            'joint_4': (-175, 175),  # entspricht -3.054 bis 3.054 Radiant
            'joint_5': (-120, 120),  # entspricht -2.094 bis 2.094 Radiant
            'joint_6': (-350, 350),  # entspricht -6.108 bis 6.108 Radiant
        }

        # Signalhandler für Ctrl+C (SIGINT) einrichten
        signal.signal(signal.SIGINT, self.signal_handler)

    def send_trajectory(self):
        while rclpy.ok():
            try:
                # Eingabe der Zielpositionen für alle Gelenke in Grad
                input_str = input(f"Geben Sie die Positionen in Grad für {', '.join(self.joint_names)} ein (getrennt durch Kommas): ")

                # Umwandlung der Eingabe in eine Liste von floats (Gradwerte)
                positions_deg = [float(x) for x in input_str.split(',')]

                # Sicherstellen, dass genau sechs Werte eingegeben wurden
                if len(positions_deg) != 6:
                    raise ValueError("Es müssen genau 6 Werte eingegeben werden.")

                # Überprüfen, ob alle Werte innerhalb der Gelenkgrenzen (in Grad) liegen
                for i, pos in enumerate(positions_deg):
                    joint_name = self.joint_names[i]
                    lower_limit, upper_limit = self.joint_limits[joint_name]
                    if not (lower_limit <= pos <= upper_limit):
                        raise ValueError(f"Position für {joint_name} muss zwischen {lower_limit}° und {upper_limit}° liegen.")

                # Umrechnung der Gradwerte in Radiant
                positions_rad = [math.radians(pos) for pos in positions_deg]

                # Erstellen und Versenden der Trajektorie
                point = JointTrajectoryPoint()
                point.positions = positions_rad
                point.time_from_start = rclpy.time.Duration(seconds=2.0).to_msg()  # Zeit bis zum Erreichen der Zielposition

                self.trajectory.points = [point]
                self.publisher.publish(self.trajectory)

                self.get_logger().info(f"Gesendete Positionen (in Grad): {positions_deg}")
                self.get_logger().info(f"Gesendete Positionen (in Radiant): {positions_rad}")

            except ValueError as e:
                self.get_logger().error(str(e))
                print("Bitte geben Sie die Positionen erneut korrekt ein.")
            except KeyboardInterrupt:
                print("Beenden mit Ctrl+C erkannt. Beende jetzt den Node...")
                break

    def signal_handler(self, sig, frame):
        # Signal-Handler für Ctrl+C, um den Node sauber zu beenden
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
        # Sicherstellen, dass rclpy sauber heruntergefahren wird
        if rclpy.ok():
            rclpy.shutdown()
        print("Node wurde sauber beendet.")

if __name__ == '__main__':
    main()

