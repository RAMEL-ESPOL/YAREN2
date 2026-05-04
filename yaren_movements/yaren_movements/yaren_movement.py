#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class YarenMotionControl(Node):
    def __init__(self):
        super().__init__('yaren_motion_control')
        
        # Publicamos al controlador de trayectoria para suavizar movimientos
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        
        self.get_logger().info("Secuencia Infinita de Movimiento Yaren Listo.")

    def send_movement(self, positions, seconds):
        """Envía un punto de trayectoria para que el robot se mueva suavemente."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        # El tiempo define qué tan lento o rápido llega (esto quita lo 'robótico')
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        
        msg.points.append(point)
        self.publisher.publish(msg)

    def run_menu(self):
        while rclpy.ok():
            try:
                number ="6";

                if number == '6':
                    print("Iniciando Rutina Larga...")
                    routine = [
                                ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 1),
                                ([-0.37, 0.0, 0.08, 0.65, 0.81, -0.18, 0.78, -0.33, -1.16, -0.22, 0.22, -1.11], 1.0),
                                ([-0.39, 0.0, 0.08, 0.65, 0.82, -0.15, 0.78, 0.32, -1.12, -0.21, 0.31, -1.95], 1.0),
                                ([-0.37, 0.0, 0.08, 0.65, 0.85, -0.17, 0.8, -0.9, -1.12, -0.21, -0.42, 0.1], 1.0),
                                ([-0.37, 0.0, 0.08, 0.65, 0.79, -0.17, 0.83, 0.26, -0.95, -0.13, 0.3, -1.42], 1.0),
                            ]
                    for pos, t in routine:
                        self.send_movement(pos, t)
                        import time
                        time.sleep(t) # Esperar a que termine cada paso

            except EOFError:
                break
def main(args=None):
    rclpy.init(args=args)
    node = YarenMotionControl()
    node.run_menu()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()