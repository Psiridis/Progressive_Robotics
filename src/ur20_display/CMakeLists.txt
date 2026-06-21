cmake_minimum_required(VERSION 3.8)
project(ur20_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()