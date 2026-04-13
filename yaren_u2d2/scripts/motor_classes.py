from math import pi
from dynamixel_sdk import * 
class Motor:
    def __init__(self, usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler, r):
        self.portHandler = portHandler
        self.r = r # En ROS 2 esto suele ser None o un Timer
        self.packetHandler = packetHandler
        self.protocol_version = 2.0 
        self.list_ids = list_ids

    def communication(self, dxl_baud_rate, addr_torque_enable):
        if not self.portHandler.is_open:
            self.portHandler.openPort()
        self.portHandler.setBaudRate(dxl_baud_rate)

    def torque(self, order, addr_torque_enable):
        for id in self.list_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, addr_torque_enable, order)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"Torque Motor {id}: {'ON' if order == 1 else 'OFF'}")

    def angleConversion(self, raw_value, to_radian_bool, id):
        # Lógica de conversión copiada de tu archivo motor_classes.py [cite: 1]
        min_v, max_v = 0, 4095
        zero_v = (max_v - min_v) // 2 + 1
        rad_span = float(180 * (pi / 180)) # 180 deg span / 2        

        if to_radian_bool:
            return 0.0 if raw_value == zero_v else ((float)(raw_value - zero_v) / (float)(zero_v)) * rad_span
        else:
            if raw_value == 0.0: return zero_v
            # Limitación por dict_range [cite: 1]
            if raw_value > self.dict_range[id][1]: raw_value = self.dict_range[id][1]
            elif raw_value < self.dict_range[id][0]: raw_value = self.dict_range[id][0]
            return int(((max_v - min_v) // 2) * (raw_value / pi) + zero_v)

class MX_motor(Motor):
    def __init__(self, usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler, r, value_max_velocity, max_min_range_dict, list_pid):
        super().__init__(usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler, r)
        self.addr_torque_enable = 64
        self.addr_goal_position = 116
        self.addr_present_position = 132
        self.dict_range = max_min_range_dict
        self.torque_enable = 1
        self.torque_disable = 0

class XCseries_motor(Motor):
    def __init__(self, usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler, r, value_max_velocity, max_min_range_dict, list_pid):
        super().__init__(usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler, r)
        self.addr_torque_enable = 64
        self.addr_goal_position = 116
        self.addr_present_position = 132
        self.dict_range = max_min_range_dict
        self.torque_enable = 1
        self.torque_disable = 0