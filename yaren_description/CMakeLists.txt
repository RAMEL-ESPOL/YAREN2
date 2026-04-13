cmake_minimum_required(VERSION 3.8)
project(yaren_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Asegúrate de que estas carpetas existan en tu paquete
install(
  DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()