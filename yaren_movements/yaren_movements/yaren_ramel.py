#!/usr/bin/env python3
""" yaren_ramel.py — Generado por yaren_pose_recorder """
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

JOINT_NAMES   = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12']
STEP_DURATION = 2
STEPS         = [[0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5]]

class RutinaNode(Node):
    def __init__(self):
        super().__init__("yaren_ramel")
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
