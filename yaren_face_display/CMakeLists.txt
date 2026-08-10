cmake_minimum_required(VERSION 3.8)
project(yaren_face_display)

# Establecer C++17 es OBLIGATORIO para usar <filesystem>
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(lifecycle_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_MIXER REQUIRED SDL2_mixer)

# -------------------------------------------------------------------
# Configuración de librerías adicionales necesarias (PThreads)
# -------------------------------------------------------------------
find_package(Threads REQUIRED)

include_directories(
  include
  src
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_MIXER_INCLUDE_DIRS}
)

add_executable(face_screen src/face_screen.cpp)

set(ROS_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  std_srvs
  lifecycle_msgs
)
ament_target_dependencies(face_screen 
rclcpp
lifecycle_msgs
${ROS_DEPENDENCIES})

# Vinculación corregida: usamos las librerías obtenidas por PkgConfig
target_link_libraries(face_screen 
  ${OpenCV_LIBS} 
  ${SDL2_LIBRARIES} 
  ${SDL2_MIXER_LIBRARIES}
  Threads::Threads
)

install(DIRECTORY
  faces
  icons
  scripts
  # Asegúrate de que esta ruta sea relativa o absoluta correcta al compilar
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  face_screen
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${ROS_DEPENDENCIES})
ament_package()