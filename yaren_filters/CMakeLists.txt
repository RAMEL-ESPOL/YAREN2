cmake_minimum_required(VERSION 3.8)
project(yaren_filters)

# ========== Configuración común ==========
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# INSTALAR TODOS LOS SCRIPTS EJECUTABLES DE PYTHON
install(PROGRAMS
  scripts/face_landmark_detector.py
  scripts/csi_cam_pub.py
  scripts/fondo_virtual.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Configuración para C++ ==========
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaren_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED) 
find_package(image_transport REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(face_filter_node 
  src/FaceFilterNode.cpp
  src/filters/GlassesFilter.cpp
  src/filters/FaceFilterBase.cpp
  src/filters/HatFilter.cpp
  src/filters/NoseFilter.cpp
  src/filters/MouthFilter.cpp
  src/filters/FaceMaskFilter.cpp
)

add_executable(animal_filter_mask
  src/AnimalFaceNode.cpp
  src/mask/AnimalFilter.cpp
)

target_include_directories(face_filter_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  include/
)

target_include_directories(animal_filter_mask PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  include/
)

ament_target_dependencies(face_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  yaren_interfaces
  cv_bridge
  message_filters
  ament_index_cpp
  image_transport
  rclcpp_lifecycle
)

ament_target_dependencies(animal_filter_mask
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  yaren_interfaces
  cv_bridge
  message_filters
  ament_index_cpp
  image_transport
  rclcpp_lifecycle
)

# INSTALAR LOS NODOS EJECUTABLES DE C++
install(TARGETS
  face_filter_node
  animal_filter_mask
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Instalación de Recursos Comunes ==========
# Aquí nos aseguramos de que "launch", "imgs" y "fondos" estén disponibles
install(DIRECTORY
  launch
  imgs
  fondos
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ========== Exportación de dependencias ==========
ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  geometry_msgs
  OpenCV
  yaren_interfaces
  cv_bridge
  message_filters
  ament_index_cpp
  image_transport
  rclcpp_lifecycle
)

ament_export_include_directories(include)

ament_package()
