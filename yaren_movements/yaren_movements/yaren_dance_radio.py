#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import random  # Importamos random para la elección inicial

class YarenDanceRadio(Node):
    def __init__(self):
        super().__init__('yaren_dance_radio')
        
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        self.get_logger().info("¡Yaren comenzó a bailar la música de la radio!")

    def send_movement(self, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        
        # Convierte segundos a formato ROS
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points.append(point)
        self.publisher.publish(msg)

    def run_dance(self):
        # Rutina 1: La coreografía larga original
        rutina_1 = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -3.0, 0.0, -1.5, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.5, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 1.0, -3.0, 0.0, -3.0, 1.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 1.0, -3.0, 0.0, -3.0, 1.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.5, 1.0, -1.5, 0.0, 0.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 1.0, -3.0, 0.5, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.5, -3.0, 0.0, -1.5, 0.0, 1.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 1.0, -3.0, 0.5, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.5, -3.0, 0.0, -1.5, 0.0, 1.0, 1.0], 2.0)
        ]

        # Rutina 2: Los nuevos movimientos que probaste en la terminal
        rutina_2 = [
            ([0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -1.5, 0.0, -3.0, 0.0, 1.5, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -1.5, 0.0, -3.0, 0.0, 1.5, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -1.5, 0.0, -3.0, 0.0, 1.5, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.5, 1.0, -1.5, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, -0.2, 0.0, 0.0, 1.3], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.3, -0.5, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, -0.2, 0.0, 0.0, 1.3], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.3, -0.5, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, -0.2, 0.0, 0.0, 1.3], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.3, -0.5, 0.0, 0.0, 0.5], 2.0)
        ]

        ultima_rutina_ejecutada = None

        # Bucle infinito
        while rclpy.ok():
            # Selección de rutina
            if ultima_rutina_ejecutada is None:
                # Si es la primera vez, elige al azar (1 o 2)
                rutina_elegida = random.choice([1, 2])
            elif ultima_rutina_ejecutada == 1:
                # Si la última fue la 1, ahora toca la 2
                rutina_elegida = 2
            else:
                # Si la última fue la 2, ahora toca la 1
                rutina_elegida = 1

            # Asignamos la lista correspondiente y mostramos en pantalla qué va a hacer
            if rutina_elegida == 1:
                self.get_logger().info("💃 Iniciando Rutina 1")
                rutina_actual = rutina_1
            else:
                self.get_logger().info("🕺 Iniciando Rutina 2")
                rutina_actual = rutina_2

            # Ejecutamos los movimientos de la rutina seleccionada
            for pos, t in rutina_actual:
                if not rclpy.ok():
                    break
                self.send_movement(pos, t)
                time.sleep(t)
            
            # Guardamos cuál fue la rutina que acaba de terminar
            ultima_rutina_ejecutada = rutina_elegida

def main(args=None):
    rclpy.init(args=args)
    node = YarenDanceRadio()
    try:
        node.run_dance()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()