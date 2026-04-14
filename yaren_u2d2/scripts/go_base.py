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

def main(args=None):
    rclpy.init(args=args)
    node = YarenMotionControl()
    node.send_movement([0.03, 0.0, -0.02, -0.02, 0.02, -0.0, 0.01, 0.34, -0.14, -0.04, 0.09, 0.33]  ,2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()