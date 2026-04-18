cmake_minimum_required(VERSION 3.5)
project(yaren_gazebo_sim)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
