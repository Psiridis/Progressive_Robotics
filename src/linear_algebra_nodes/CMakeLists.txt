cmake_minimum_required(VERSION 3.8)
project(linear_algebra_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(linear_algebra_service REQUIRED)

# Server node
#Binary creation from source file(s)
add_executable(server_node src/server_node.cpp)
#Link the target (server_node) to the libraries.
ament_target_dependencies(server_node rclcpp geometry_msgs linear_algebra_service)

# Client node
#Binary creation from source file(s)
add_executable(client_node src/client_node.cpp)
#Link the target (client_node) to the libraries.
ament_target_dependencies(client_node rclcpp geometry_msgs linear_algebra_service)

# Install targets into the colcon install tree
# This is what allows 
# ros2 run linear_algebra_nodes server_node and 
# ros2 run linear_algebra_nodes client_node
# to find the executables after "source install/setup.bash" has been run.   
install(TARGETS
  server_node
  client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
