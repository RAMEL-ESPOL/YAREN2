#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import *
from motor_classes import XCseries_motor, MX_motor
import time

class U2D2CommunicationNode(Node):
    def __init__(self):
        super().__init__('yaren_motor_communication')
        
        # Declaración de parámetros (Equivalente a rospy.get_param)
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('dxl_baud_rate', 1000000)
        
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        dxl_baud_rate = self.get_parameter('dxl_baud_rate').get_parameter_value().integer_value

        self.num_joints = 12
        self.portHandler = PortHandler(usb_port)
        self.packetHandler = PacketHandler(2.0)

        # Inicialización de motores (Lógica de u2d2_communication.py original)
        self.init_motors(usb_port, dxl_baud_rate)

        # Publicador de estados (JointState)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Suscriptor a metas (JointState)
        self.create_subscription(JointState, '/joint_goals', self.set_positions_callback, 10)

        # Timer para publicar el estado de los motores periódicamente (10Hz)
        self.timer = self.create_timer(0.1, self.joint_state_publisher)

        self.get_logger().info("Nodo u2d2_communication iniciado correctamente")

    def init_motors(self, usb_port, dxl_baud_rate):
        # Mapeo idéntico al que tenías en ROS 1
        # dummy_r se pasa como None ya que no usamos rospy.Rate
        dummy_r = None
        
        self.neck = XCseries_motor(usb_port, dxl_baud_rate, [3,4], self.portHandler, self.packetHandler, dummy_r, 20, {3:[-1,1],4:[-1,1]}, {3:[100,0,50],4:[100,0,50]})
        self.sh_r = XCseries_motor(usb_port, dxl_baud_rate, [5], self.portHandler, self.packetHandler, dummy_r, 20, {5:[-1,1]}, {5:[100,0,50]})
        self.fa_r = XCseries_motor(usb_port, dxl_baud_rate, [6,7], self.portHandler, self.packetHandler, dummy_r, 20, {6:[-1,1],7:[-1,1]}, {6:[100,0,50],7:[100,0,50]})
        self.el_r = XCseries_motor(usb_port, dxl_baud_rate, [8], self.portHandler, self.packetHandler, dummy_r, 20, {8:[-1,1]}, {8:[100,0,50]})
        self.sh_l = XCseries_motor(usb_port, dxl_baud_rate, [9], self.portHandler, self.packetHandler, dummy_r, 20, {9:[-1,1]}, {9:[100,0,50]})
        self.fa_l = XCseries_motor(usb_port, dxl_baud_rate, [10,11], self.portHandler, self.packetHandler, dummy_r, 20, {10:[-1,1],11:[-1,1]}, {10:[100,0,50],11:[100,0,50]})
        self.el_l = XCseries_motor(usb_port, dxl_baud_rate, [12], self.portHandler, self.packetHandler, dummy_r, 20, {12:[-1,1]}, {12:[100,0,50]})
        self.trunk = MX_motor(usb_port, dxl_baud_rate, [1], self.portHandler, self.packetHandler, dummy_r, 20, {1:[-1,1]}, {1:[100,0,50]})
        self.hip = MX_motor(usb_port, dxl_baud_rate, [2], self.portHandler, self.packetHandler, dummy_r, 20, {2:[-1,1]}, {2:[100,0,50]})

        self.list_motors = [self.neck, self.sh_r, self.fa_r, self.el_r, self.sh_l, self.fa_l, self.el_l, self.trunk, self.hip]

    def set_positions_callback(self, msg):
        # Lógica para enviar posiciones a los motores (SyncWrite no implementado para mantener paridad con tu script)
        for motor in self.list_motors:
            for dxl_id in motor.list_ids:
                if len(msg.position) >= dxl_id:
                    # Conversion de rad a ticks (False = to_radian_bool off)
                    new_angle = motor.angleConversion(msg.position[dxl_id-1], False, dxl_id)
                    self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, motor.addr_goal_position, int(new_angle))

    def joint_state_publisher(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i+1}' for i in range(self.num_joints)]
        
        positions = [0.0] * self.num_joints
        
        for motor in self.list_motors:
            for dxl_id in motor.list_ids:
                # Leer posición actual (addr 132)
                dxl_pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
                if res == COMM_SUCCESS:
                    # Conversión de ticks a rad (True = to_radian_bool on)
                    positions[dxl_id-1] = motor.angleConversion(dxl_pos, True, dxl_id)
        
        msg.position = positions
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = U2D2CommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()