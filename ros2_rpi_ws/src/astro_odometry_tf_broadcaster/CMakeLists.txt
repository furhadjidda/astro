cmake_minimum_required(VERSION 3.8)
project(astro_odometry_tf_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odometry_tf_broadcaster src/odometry_tf_broadcaster.cpp)

ament_target_dependencies(odometry_tf_broadcaster geometry_msgs tf2 tf2_ros nav_msgs rclcpp)

install(TARGETS
  odometry_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
