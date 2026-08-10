cmake_minimum_required(VERSION 3.8)
project(yaren_arm_mimic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_action      REQUIRED)
find_package(rclpy              REQUIRED)
find_package(yaren_interfaces   REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(control_msgs       REQUIRED)
find_package(trajectory_msgs    REQUIRED)   # ← faltaba

add_executable(body_tracker_node src/body_tracker_node.cpp)
ament_target_dependencies(body_tracker_node
  rclcpp sensor_msgs yaren_interfaces std_msgs)
target_include_directories(body_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(yaren_controller src/yaren_controller.cpp)
ament_target_dependencies(yaren_controller
  rclcpp rclcpp_action control_msgs trajectory_msgs
  sensor_msgs yaren_interfaces std_msgs)
target_include_directories(yaren_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/csi_cam_pub.py
  scripts/body_points_detector.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS body_tracker_node yaren_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclpy rclcpp_action control_msgs trajectory_msgs
  sensor_msgs yaren_interfaces std_msgs)

ament_package()