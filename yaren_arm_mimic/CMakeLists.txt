cmake_minimum_required(VERSION 3.8)
project(yaren_arm_mimic)

# ========== Configuración para C++ ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaren_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Configuración del ejecutable C++
add_executable(body_tracker_node src/body_tracker_node.cpp)
ament_target_dependencies(body_tracker_node 
  rclcpp 
  sensor_msgs 
  yaren_interfaces 
  std_msgs
)

add_executable(yaren_controller src/yaren_controller.cpp)
ament_target_dependencies(yaren_controller 
  rclcpp 
  rclcpp_action
  control_msgs
  sensor_msgs 
  yaren_interfaces 
  std_msgs
)

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Instalar el script Python

install(PROGRAMS
  scripts/csi_cam_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/body_points_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Instalación común ==========
install(DIRECTORY
  include
  launch
  scripts
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  yaren_controller
  body_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

ament_export_dependencies(
  rclcpp
  rclpy
  rclcpp_action
  control_msgs
  sensor_msgs
  yaren_interfaces
  std_msgs
)

ament_export_include_directories(include)
ament_package()