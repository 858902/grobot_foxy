################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(grobot_navigation2)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

## EXPORTS
include_directories(include)
include_directories(${ament_INCLUDE_DIRS})



################################################################################
# Build
################################################################################
add_executable(admittance_interfaces src/admittance_interfaces.cpp)  
ament_target_dependencies(admittance_interfaces rclcpp std_srvs orocos_kdl sensor_msgs Eigen3 tf2 kdl_parser gazebo_msgs tf2_geometry_msgs std_msgs geometry_msgs nav2_msgs)

add_executable(waypoint_follower src/waypoint_follower.cpp)  
ament_target_dependencies(waypoint_follower rclcpp std_srvs  sensor_msgs  tf2  gazebo_msgs tf2_geometry_msgs std_msgs geometry_msgs nav2_msgs ament_index_cpp yaml-cpp)

add_executable(waypoint_planning src/waypoint_planning.cpp)  
ament_target_dependencies(waypoint_planning rclcpp std_srvs  sensor_msgs  tf2  gazebo_msgs tf2_geometry_msgs std_msgs geometry_msgs nav2_msgs)

add_executable(waypoint_dynamic_programming src/waypoint_dynamic_programming.cpp)  
ament_target_dependencies(waypoint_dynamic_programming rclcpp std_srvs  sensor_msgs  tf2 std_msgs yaml-cpp ament_index_cpp)

target_link_libraries(waypoint_dynamic_programming
  ${rclcpp_LIBRARIES}
  yaml-cpp
)

target_link_libraries(waypoint_follower
  ${rclcpp_LIBRARIES}
  yaml-cpp
)
################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  admittance_interfaces
  waypoint_follower
  waypoint_planning
  waypoint_dynamic_programming
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# Macro for ament package
################################################################################
ament_package()