cmake_minimum_required(VERSION 3.8)
project(yaren_face_display)

# 1. Encontrar todos los paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 2. Configurar los directorios de inclusión
include_directories(
  include
  src
  /opt/ros/humble/include           # <--- AGREGA ESTA LÍNEA MANUALMENTE
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}  # <--- ASEGÚRATE DE QUE ESTO ESTÉ AQUÍ
)
include_directories(
  include
  /opt/ros/humble/include/cv_bridge  # <--- RUTA FORZADA
)

# 3. Crear los ejecutables
add_executable(face_screen src/face_screen.cpp)
add_executable(face_cambiar src/face_cambiar.cpp)
add_executable(face_hablar src/face_hablar.cpp)
add_executable(face_hablar_cambiar src/face_hablar_cambiar.cpp)


# 4. Vincular TODAS las dependencias de ROS 2 a face_screen
ament_target_dependencies(face_screen 
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)

# Vincular las dependencias de ROS 2 a face_cambiar
ament_target_dependencies(face_cambiar 
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)
# --------------------------------

# Vincular las dependencias de ROS 2 a face_hablar
ament_target_dependencies(face_hablar 
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)
# Vincular las dependencias de ROS 2 a face_hablar
ament_target_dependencies(face_hablar_cambiar 
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)
# --------------------------------
# 5. Vincular las librerías de OpenCV a ambos
target_link_libraries(face_screen ${OpenCV_LIBS})
target_link_libraries(face_cambiar ${OpenCV_LIBS}) # Faltaba esto también
target_link_libraries(face_hablar ${OpenCV_LIBS}) # Faltaba esto también
target_link_libraries(face_hablar_cambiar ${OpenCV_LIBS}) # Faltaba esto también


# 6. Instalar los archivos de datos
install(DIRECTORY
  faces
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 7. Instalar los ejecutables
install(TARGETS 
  face_screen
  face_cambiar
  face_hablar
  face_hablar_cambiar
  DESTINATION lib/${PROJECT_NAME}
)

# 8. Exportar las dependencias del paquete
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
  cv_bridge
)
ament_export_include_directories(include)

ament_package()