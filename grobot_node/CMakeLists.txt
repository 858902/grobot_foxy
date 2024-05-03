################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(grobot_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
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

find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(laser_geometry REQUIRED)


find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(ament_index_cpp REQUIRED)

## EXPORTS
include_directories(include)
include_directories(${ament_INCLUDE_DIRS})

################################################################################
# Build
################################################################################
add_executable(lidar_calibration src/lidar_calibration.cpp)  
ament_target_dependencies(lidar_calibration rclcpp std_srvs  sensor_msgs  tf2 tf2_ros tf2_sensor_msgs gazebo_msgs tf2_geometry_msgs std_msgs geometry_msgs nav2_msgs ament_index_cpp yaml-cpp)

add_executable(velocity_manager src/velocity_manager.cpp)  
ament_target_dependencies(velocity_manager rclcpp std_srvs  sensor_msgs  gazebo_msgs std_msgs geometry_msgs nav2_msgs ament_index_cpp)

add_executable(force_publisher_node src/force_publisher_node.cpp)  
ament_target_dependencies(force_publisher_node rclcpp std_srvs  sensor_msgs std_msgs ament_index_cpp)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lidar_calibration
  velocity_manager
  force_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# Macro for ament package
################################################################################
ament_package()
