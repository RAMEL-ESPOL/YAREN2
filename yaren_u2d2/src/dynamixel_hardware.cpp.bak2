#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cmath>
#include <algorithm>

#define DEVICE_NAME "/dev/ttyUSB0" 
#define PROTOCOL_VERSION 2.0 
#define BAUDRATE 1000000 

using hardware_interface::CallbackReturn;
using hardware_interface::HardwareInfo;
using hardware_interface::SystemInterface;
using hardware_interface::return_type;
using namespace std::chrono_literals;

namespace yaren_hw_interface
{

class YarenSystemPositionOnly : public SystemInterface
{
public:
  CallbackReturn on_init(const HardwareInfo & info) override
  {
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dxl_baud_rate_ = BAUDRATE;


    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(rclcpp::get_logger("YarenSystemPositionOnly"), "Failed to open USB port!");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("YarenSystemPositionOnly"), "USB port opened successfully!");
    }

    if (!portHandler_->setBaudRate(dxl_baud_rate_)) {
      RCLCPP_ERROR(rclcpp::get_logger("YarenSystemPositionOnly"), "Failed to set baud rate!");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("YarenSystemPositionOnly"), "Baudrate set successfully!");
    }

    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    for (const auto & joint : info.joints)
    {
      joint_names_.push_back(joint.name);
      position_.push_back(0.0);
      velocity_.push_back(0.0);
      effort_.push_back(0.0);
      position_commands_.push_back(0.0);
    }

    portHandler_->openPort();
    portHandler_->setBaudRate(dxl_baud_rate_);

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      int id = std::stoi(info_.joints[i].parameters["id"]);
      double min = std::stod(info_.joints[i].parameters["min"]);
      double max = std::stod(info_.joints[i].parameters["max"]);
      joint_id_map_[joint_names_[i]] = id;
      joint_limits_[joint_names_[i]] = std::make_pair(min, max);
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // BYPASS: Ignorar configuración inicial de PID y Torque para joint_2
      if (joint_names_[i] == "joint_2") {
        continue; 
      }

      int id = joint_id_map_[joint_names_[i]];
      uint8_t dxl_error = 0;

      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 84, 100, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("YarenSystemPositionOnly"), "Error configurando parte P del PID en ID %d", id);
      }

      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 82, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("YarenSystemPositionOnly"), "Error configurando parte I del PID en ID %d", id);
      }

      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 80, 50, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("YarenSystemPositionOnly"), "Error configurando parte D del PID en ID %d", id);
      }

      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("YarenSystemPositionOnly"), "Error activando torque en ID %d", id);
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {

      state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_[i]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
    return command_interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // BYPASS: No leer físicamente el motor que no existe
      if (joint_names_[i] == "joint_2") {
        // Mantenemos el joint lógico inalterado copiando su propio comando
        position_[i] = position_commands_[i];
        continue;
      }

      int id = joint_id_map_[joint_names_[i]];
      uint32_t dxl_present_position;
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, 132, &dxl_present_position);
      
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("YarenSystemPositionOnly"), "Failed to read position for joint %s", joint_names_[i].c_str());
      } else if (dxl_error != 0) {
        RCLCPP_INFO(rclcpp::get_logger("YarenSystemPositionOnly"), "Dynamixel error for joint %s: %d", joint_names_[i].c_str(), dxl_error);
      } else {
        position_[i] = convertTicksToRadians(dxl_present_position, joint_names_[i]);
      }
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    dynamixel::GroupSyncWrite sync_write(portHandler_, packetHandler_, 116, 4); 

    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
      // BYPASS: No enviar comandos de escritura al motor que no existe
      if (joint_names_[i] == "joint_2") {
        continue;
      }

      int32_t goal = convertRadiansToTicks(position_commands_[i], joint_names_[i]);
      sync_write.addParam(joint_id_map_[joint_names_[i]], (uint8_t*)&goal);
    }
    sync_write.txPacket();
    sync_write.clearParam();

    return return_type::OK;
  }

private:
  double convertTicksToRadians(uint32_t tick, const std::string& joint_name)
  {
    if (joint_name == "joint_8" || joint_name == "joint_12") {
      return (static_cast<double>(tick)) * (M_PI / 2048.0) - 1.75;
    } else {
      return (static_cast<double>(tick) - 2048.0) * (2.0 * M_PI / 4096.0);
    }
  }

  int32_t convertRadiansToTicks(double rad, const std::string& joint_name)
  {
    auto limits = joint_limits_[joint_name];
    if (rad < limits.first) {
      rad = limits.first;
    } else if (rad > limits.second) {
      rad = limits.second;
    }
    if (joint_name == "joint_8" || joint_name == "joint_12") {
      return static_cast<int>((rad + 1.75) * 2048.0 / M_PI);   
    } else {
      return static_cast<int32_t>(rad * 4096.0 / (2.0 * M_PI) + 2048.0);   
    }
  }

  std::vector<std::string> joint_names_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> position_commands_;

  std::map<std::string, int> joint_id_map_;
  std::map<std::string, std::pair<double, double>> joint_limits_;
  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
  int dxl_baud_rate_;
};

} // namespace yaren_hw_interface

PLUGINLIB_EXPORT_CLASS(yaren_hw_interface::YarenSystemPositionOnly, hardware_interface::SystemInterface)