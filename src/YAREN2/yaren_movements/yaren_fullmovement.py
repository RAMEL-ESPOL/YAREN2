#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class YarenMotionControl(Node):
    def __init__(self):
        super().__init__('yaren_motion_control')
        
        # Publicador al controlador de trayectoria
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        
        self.get_logger().info("Controlador de Movimiento Yaren Iniciado (Modo Infinito).")

    def send_movement(self, positions, seconds):
        """Envía un punto de trayectoria para que el robot se mueva."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        # Conversión de tiempo para la duración del movimiento
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        
        msg.points.append(point)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YarenMotionControl()

    # Lista de posiciones (Posición, Tiempo)
    rutina_6 = [
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),
        ([-0.01, -0.01, -0.0, 0.35, 0.59, -0.19, -0.2, -0.07, -0.75, -0.2, 0.33, -0.01], 1.0),
        ([-0.01, 0.0, -0.0, 0.35, 0.78, -0.04, -1.08, 0.21, -0.88, -0.1, 1.13, 0.21], 1.0),
        ([-0.01, 0.21, -0.01, 0.35, 0.37, 0.13, -1.01, 0.1, -0.52, -0.1, 0.81, 0.13], 1.0),
        ([0.0, -0.21, -0.01, 0.35, 0.52, 0.01, -0.96, 0.06, -0.23, -0.13, 0.83, -0.12], 1.0),
        ([0.03, 0.01, -0.01, 0.35, 0.94, 0.03, -0.9, 0.08, -0.36, 0.0, 0.86, 0.11], 1.0),
        ([0.01, 0.02, -0.01, 0.35, 0.58, -0.02, -1.09, 0.03, -0.85, -0.03, 0.78, 0.19], 1.0),
        ([0.05, -0.02, -0.01, 0.35, 0.6, 0.08, 0.28, -0.03, -0.81, -0.04, -0.32, 0.02], 1.0),
        ([-0.04, -0.02, -0.01, 0.35, 1.02, -0.25, -1.3, -0.28, -1.07, -0.15, 0.96, -0.24], 1.0),
        ([-0.04, 0.21, -0.02, 0.35, 0.8, -0.25, -1.3, -0.26, -0.9, -0.16, 0.97, -0.21], 1.0),
        ([-0.04, -0.21, -0.02, 0.35, 0.75, -0.25, -1.29, -0.26, -0.87, -0.16, 0.97, -0.22], 1.0),
        ([0.0, 0.15, -0.03, 0.35, 0.58, -0.14, 0.41, -0.03, -0.04, -0.05, 0.76, -0.0], 1.0),
        ([0.04, -0.15, -0.03, 0.35, 0.46, -0.19, 0.34, 0.03, -0.81, -0.15, -0.37, 0.04], 1.0),
        ([-0.01, -0.01, -0.0, 0.35, 0.59, -0.19, -0.2, -0.07, -0.75, -0.2, 0.33, -0.01], 0.3),
        ([-0.01, 0.0, -0.0, 0.35, 0.78, -0.04, -1.08, 0.21, -0.88, -0.1, 1.13, 0.21], 0.5),
        ([-0.01, 0.21, -0.01, 0.35, 0.37, 0.13, -1.01, 0.1, -0.52, -0.1, 0.81, 0.13], 0.5),
        ([0.0, -0.21, -0.01, 0.35, 0.52, 0.01, -0.96, 0.06, -0.23, -0.13, 0.83, -0.12], 0.5),
        ([0.03, 0.01, -0.01, 0.35, 0.94, 0.03, -0.9, 0.08, -0.36, 0.0, 0.86, 0.11], 0.5),
        ([0.01, 0.02, -0.01, 0.35, 0.58, -0.02, -1.09, 0.03, -0.85, -0.03, 0.78, 0.19], 0.5),
        ([0.05, -0.02, -0.01, 0.35, 0.6, 0.08, 0.28, -0.03, -0.81, -0.04, -0.32, 0.02], 0.5),
        ([-0.04, -0.02, -0.01, 0.35, 1.02, -0.25, -1.3, -0.28, -1.07, -0.15, 0.96, -0.24], 0.5),
        ([-0.04, 0.21, -0.02, 0.35, 0.8, -0.25, -1.3, -0.26, -0.9, -0.16, 0.97, -0.21], 0.5),
        ([-0.04, -0.21, -0.02, 0.35, 0.75, -0.25, -1.29, -0.26, -0.87, -0.16, 0.97, -0.22], 0.5),
        ([0.0, 0.15, -0.03, 0.35, 0.58, -0.14, 0.41, -0.03, -0.04, -0.05, 0.76, -0.0], 0.5),
        ([0.04, -0.15, -0.03, 0.35, 0.46, -0.19, 0.34, 0.03, -0.81, -0.15, -0.37, 0.04], 0.5),
        ([-0.01, -0.08, -0.02, 0.35, 0.05, -0.13, 0.04, -0.17, -0.34, 0.08, 0.04, -0.13], 1.0)
    ]

    try:
        # BUCLE INFINITO
        while rclpy.ok():
            for i, (pos, t) in enumerate(rutina_6):
                node.get_logger().info(f"Ejecutando paso {i+1}/{len(rutina_6)}")
                node.send_movement(pos, t)
                # Esperamos la duración del movimiento para no solapar comandos
                time.sleep(t) 
            
            node.get_logger().info("--- Reiniciando rutina ---")
            
    except KeyboardInterrupt:
        node.get_logger().info("Movimiento detenido por el usuario.")
    finally:
        # Apagado limpio
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()