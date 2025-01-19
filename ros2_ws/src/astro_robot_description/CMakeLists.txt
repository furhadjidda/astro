cmake_minimum_required(VERSION 3.8)
project(astro_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
