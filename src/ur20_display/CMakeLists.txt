cmake_minimum_required(VERSION 3.8)
project(ur20_display)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

# Build ur20_display_node executable
add_executable(ur20_display_node src/ur20_display_node.cpp)

target_include_directories(ur20_display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ur20_display_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
)

target_link_libraries(ur20_display_node Eigen3::Eigen fmt::fmt)

install(TARGETS ur20_display_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()