#!/usr/bin/env python3
""" rutina_17052026_003702.py — Generado por yaren_pose_recorder """
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

JOINT_NAMES   = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12']
STEP_DURATION = 2
STEPS         = [[0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.5340057267032992, -3.0, 0.0, 0.33681963529237224, 0.0], [0.0, 0.0, 0.0, 0.0, 1.365161573207532, 1.0, 0.0, 1.0, -3.0, 0.0, 1.555022055744807, 0.0], [0.0, 0.0, 0.0, 0.0, 1.3510390962629508, 1.0, 0.0, 0.7303611930860867, -3.0, 0.0, 1.6599062610617792, 0.0], [0.0, 0.0, 0.0, 0.0, 1.2814978349304746, 1.0, 0.0, 1.0, -3.0, 0.0, 0.43909480826842096, 0.0], [0.0, 0.0, 0.0, 0.0, 2.243245732023008, 0.5291987887950994, 0.0, 0.0, -3.0, 0.0, 2.6886469074816373, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0670237567803671, 1.0, 0.0, 0.512360984812221, -3.0, 0.0, 0.5635576418132744, 0.0], [0.0, 0.0, 0.0, 0.0, 2.7276425100592316, 0.1904597831753625, 0.0, 0.23684211043687908, -2.942502668315569, 0.04020792425484697, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.18709971486434407, 1.0, 0.0, 0.678099386433399, -3.0, 0.0, 0.0, 0.0]]

class RutinaNode(Node):
    def __init__(self):
        super().__init__("rutina_17052026_003702")
        self._pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory", 10)

    def run(self):
        self.get_logger().info("Esperando controlador...")
        timeout = 10.0
        start   = time.time()
        while self._pub.get_subscription_count() == 0:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout: controlador no encontrado")
                return
            time.sleep(0.1)

        self.get_logger().info("Controlador conectado. Ejecutando rutina...")
        time.sleep(0.5)

        for i, positions in enumerate(STEPS):
            self.get_logger().info(f"Paso {i+1}/{len(STEPS)}")
            msg              = JointTrajectory()
            msg.joint_names  = JOINT_NAMES
            pt               = JointTrajectoryPoint()
            pt.positions     = positions
            pt.velocities    = [0.0] * len(JOINT_NAMES)
            pt.time_from_start = Duration(sec=STEP_DURATION, nanosec=0)
            msg.points       = [pt]
            self._pub.publish(msg)
            time.sleep(STEP_DURATION + 0.5)

        self.get_logger().info("Rutina completada.")

def main():
    rclpy.init()
    node = RutinaNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": main()
