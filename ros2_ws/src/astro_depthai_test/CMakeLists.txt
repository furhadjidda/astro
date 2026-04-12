cmake_minimum_required(VERSION 3.8)
project(astro_depthai_test)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
