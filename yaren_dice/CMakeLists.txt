cmake_minimum_required(VERSION 3.8)
project(yaren_dice)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaren_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)   # <-- AGREGAR
find_package(YAML-CPP REQUIRED)

install(PROGRAMS
  scripts/body_landmarks_visual.py
  scripts/body_landmarks.py
  scripts/speaker_node.py
  scripts/csi_cam_pub.py
DESTINATION lib/${PROJECT_NAME}
)

add_executable(game_manager src/game_manager.cpp)
ament_target_dependencies(game_manager 
  rclcpp 
  yaren_interfaces 
  std_msgs
  trajectory_msgs    # <-- AGREGAR
)
target_link_libraries(game_manager ${YAML_CPP_LIBRARIES})
target_include_directories(game_manager PRIVATE ${YAML_CPP_INCLUDE_DIRS})

add_executable(pose_detector src/pose_detector.cpp)
ament_target_dependencies(pose_detector 
  rclcpp 
  yaren_interfaces 
  std_msgs
)

find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)

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
  trajectory_msgs    # <-- AGREGAR
  YAML-CPP
)

ament_export_include_directories(include)
ament_package()