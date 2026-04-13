#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

# La lista de rutina se mantiene igual
routine = [
    ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 5.25),
    ([-0.01, -0.01, -0.0, 0.35, 0.59, -0.19, -0.2, -0.07, -0.75, -0.2, 0.33, -0.01], 2),
    ([-0.01, 0.0, -0.0, 0.35, 0.78, -0.04, -1.08, 0.21, -0.88, -0.1, 1.13, 0.21], 2),
    ([-0.01, 0.21, -0.01, 0.35, 0.37, 0.13, -1.01, 0.1, -0.52, -0.1, 0.81, 0.13], 2),
    ([0.0, -0.21, -0.01, 0.35, 0.52, 0.01, -0.96, 0.06, -0.23, -0.13, 0.83, -0.12], 2),
    ([0.03, 0.01, -0.01, 0.35, 0.94, 0.03, -0.9, 0.08, -0.36, 0.0, 0.86, 0.11], 2),
    ([0.01, 0.02, -0.01, 0.35, 0.58, -0.02, -1.09, 0.03, -0.85, -0.03, 0.78, 0.19], 2),
    ([0.05, -0.02, -0.01, 0.35, 0.6, 0.08, 0.28, -0.03, -0.81, -0.04, -0.32, 0.02], 2),
    ([-0.04, -0.02, -0.01, 0.35, 1.02, -0.25, -1.3, -0.28, -1.07, -0.15, 0.96, -0.24], 2),
    ([-0.04, 0.21, -0.02, 0.35, 0.8, -0.25, -1.3, -0.26, -0.9, -0.16, 0.97, -0.21], 2),
    ([-0.04, -0.21, -0.02, 0.35, 0.75, -0.25, -1.29, -0.26, -0.87, -0.16, 0.97, -0.22], 2),
    ([0.0, 0.15, -0.03, 0.35, 0.58, -0.14, 0.41, -0.03, -0.04, -0.05, 0.76, -0.0], 2),
    ([0.04, -0.15, -0.03, 0.35, 0.46, -0.19, 0.34, 0.03, -0.81, -0.15, -0.37, 0.04], 2),
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
    ([-0.01, -0.08, -0.02, 0.35, 0.05, -0.13, 0.04, -0.17, -0.34, 0.08, 0.04, -0.13], 2),
]

class JointGoalRepeater(Node):
    def __init__(self):
        super().__init__('joint_goal_repeater')
        self.publisher_ = self.create_publisher(JointState, '/joint_goals', 10)
        self.get_logger().info("Ejecutando rutina infinita en ROS 2...")
        
        # Preparamos el mensaje
        self.joint_state = JointState()
        self.joint_state.name = [f"joint_{i}" for i in range(1, 13)]

    def run_routine(self):
        try:
            while rclpy.ok():
                for pose, wait_time in routine:
                    if not rclpy.ok():
                        break
                    
                    # Actualizar mensaje
                    self.joint_state.header.stamp = self.get_clock().now().to_msg()
                    self.joint_state.position = [float(p) for p in pose] # ROS2 es estricto con floats
                    
                    # Publicar
                    self.publisher_.publish(self.joint_state)
                    self.get_logger().info(f"Enviado: {pose} | Esperando {wait_time}s")
                    
                    # Dormir (time.sleep es aceptable aquí para rutinas bloqueantes simples)
                    time.sleep(wait_time)
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = JointGoalRepeater()
    
    # Ejecutamos la rutina
    node.run_routine()

    # Limpieza al cerrar
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()