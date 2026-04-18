cmake_minimum_required(VERSION 3.8)
project(yaren_interfaces)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Combina todos los archivos de mensajes en una sola llamada
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BodyPoints.msg"
  "msg/BodyPosition.msg"
  "msg/Landmarks.msg"
  "msg/PersonResponse.msg"
  "msg/PoseResult.msg"
  "action/ProcessResponse.action"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES std_msgs
)

ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY action msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()