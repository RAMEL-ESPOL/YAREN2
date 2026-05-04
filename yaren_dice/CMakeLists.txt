cmake_minimum_required(VERSION 3.8)
project(yaren_dice)

# ========== Configuración para C++ ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaren_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(YAML-CPP REQUIRED)



# Asegúrate de que esta sección apunte a la carpeta real donde está el .py
install(PROGRAMS
  scripts/body_landmarks.py
  scripts/speaker_node.py
  scripts/csi_cam_pub.py  # <-- Debe estar aquí
  DESTINATION lib/${PROJECT_NAME}
)

# Configuración del ejecutable C++
add_executable(game_manager src/game_manager.cpp)
ament_target_dependencies(game_manager 
  rclcpp 
  yaren_interfaces 
  std_msgs
)

target_link_libraries(game_manager ${YAML_CPP_LIBRARIES})
target_include_directories(game_manager PRIVATE ${YAML_CPP_INCLUDE_DIRS})
#[[
add_executable(yaren_face_display src/face_screen.cpp)
ament_target_dependencies(face_screen
  rclcpp
  std_msgs
  sensor_msgs
  yaren_interfaces
)
]]

add_executable(pose_detector src/pose_detector.cpp)
ament_target_dependencies(pose_detector 
  rclcpp 
  yaren_interfaces 
  std_msgs
)

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)


# ========== Instalación común ==========
install(DIRECTORY
  config
  include
  launch
  models
  scripts
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  game_manager
  pose_detector
  #face_screen
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

ament_export_dependencies(
  rclpy
  rclcpp_action
  control_msgs
  sensor_msgs
  yaren_interfaces
  std_msgs
  YAML-CPP
)

ament_export_include_directories(include)
ament_package()
