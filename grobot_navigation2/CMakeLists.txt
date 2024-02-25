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

################################################################################
# Build
################################################################################
add_executable(admittance_interfaces src/admittance_interfaces.cpp)  
ament_target_dependencies(admittance_interfaces rclcpp std_srvs orocos_kdl sensor_msgs Eigen3 tf2 kdl_parser gazebo_msgs tf2_geometry_msgs std_msgs geometry_msgs nav2_msgs)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  admittance_interfaces
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
