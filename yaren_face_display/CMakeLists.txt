cmake_minimum_required(VERSION 3.8)
project(yaren_face_display)

# -------------------------------------------------------------------
# 1. ENCONTRAR TODOS LOS PAQUETES NECESARIOS
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(lifecycle_msgs REQUIRED)
# Usamos PkgConfig para SDL2 y SDL2_mixer (Evita el error de CMake de Ubuntu)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_MIXER REQUIRED SDL2_mixer)

# -------------------------------------------------------------------
# 2. CONFIGURAR LOS DIRECTORIOS DE INCLUSIÓN
# -------------------------------------------------------------------
include_directories(
  include
  src
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_MIXER_INCLUDE_DIRS}
)

# -------------------------------------------------------------------
# 3. CREAR LOS EJECUTABLES
# -------------------------------------------------------------------
add_executable(face_screen src/face_screen.cpp)

# -------------------------------------------------------------------
# 4. VINCULAR DEPENDENCIAS DE ROS 2 (ament)
# -------------------------------------------------------------------
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
ament_target_dependencies(face_screen ${ROS_DEPENDENCIES})


# -------------------------------------------------------------------
# 5. VINCULAR LIBRERÍAS DE OPENCV Y SDL2/MIXER NATIVAS
# -------------------------------------------------------------------
# Vinculamos OpenCV y el backend de audio de SDL2 a los objetivos reales
target_link_libraries(face_screen ${OpenCV_LIBS} ${SDL2_LIBRARIES} ${SDL2_MIXER_LIBRARIES})


# -------------------------------------------------------------------
# 6. INSTALAR LOS ARCHIVOS DE DATOS Y RECURSOS
# -------------------------------------------------------------------
# Asegúrate de crear e incluir tu carpeta "audio" aquí si guardas tus mp3 ahí dentro
install(DIRECTORY
  faces
  icons
  scripts
  ../yaren_radio/audios
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# 7. INSTALAR LOS EJECUTABLES EN EL ESPACIO DE TRABAJO
# -------------------------------------------------------------------
install(TARGETS 
  face_screen
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# 8. EXPORTAR LAS DEPENDENCIAS DEL PAQUETE
# -------------------------------------------------------------------
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
  cv_bridge
  std_srvs
  lifecycle_msgs
)
ament_export_include_directories(include)

ament_package()