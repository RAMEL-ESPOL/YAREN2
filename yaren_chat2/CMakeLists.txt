cmake_minimum_required(VERSION 3.8)
project(yaren_chat2)

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(yaren_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# C++ ejecutable
add_executable(control_manager_node src/control_manager_node.cpp)
ament_target_dependencies(control_manager_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  lifecycle_msgs
)

include_directories(
  include
  /usr/local/include
)

# Instalar scripts Python
install(PROGRAMS
  scripts/llm_local_lifecycle_node.py
  scripts/stt_local_lifecycle_node.py    # NUEVO
  scripts/tts_local_lifecycle_node.py    # NUEVO
  scripts/utils/text_processor.py
  scripts/utils/react_state.py
  scripts/utils/movement_agent.py
  scripts/utils/state_graph.py
  scripts/utils/config.py
  DESTINATION lib/${PROJECT_NAME}
)
# Instalar directorios
install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

# Instalar target C++
install(TARGETS
  control_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Exportar dependencias
ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  yaren_interfaces
  rclcpp_lifecycle
  rclcpp_action
  lifecycle_msgs
)

ament_export_include_directories(include)
ament_package()
