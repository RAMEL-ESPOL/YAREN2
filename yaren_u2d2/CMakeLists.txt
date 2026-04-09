cmake_minimum_required(VERSION 3.5)
project(yaren_u2d2)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# --- CONFIGURACIÓN C++ (Hardware Interface) ---
add_library(yaren_hardware_interface SHARED
  src/dynamixel_hardware.cpp
)

ament_target_dependencies(yaren_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  sensor_msgs
  std_msgs
  dynamixel_sdk
)

target_link_libraries(yaren_hardware_interface
    ${dynamixel_sdk_LIBRARIES}
)

# Registrar el archivo XML del plugin
pluginlib_export_plugin_description_file(hardware_interface yaren_hardware_plugin.xml)

# --- INSTALACIÓN ---

# Instalar librería C++
install(
  TARGETS yaren_hardware_interface
  EXPORT export_yaren_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instalar Scripts de Python (IMPORTANTE)
install(PROGRAMS
  scripts/motor_classes.py
  scripts/u2d2_communication.py
  scripts/w_datos_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar directorios de configuración y lanzamiento
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar el XML del plugin
install(
  FILES yaren_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(yaren_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib)

ament_package()