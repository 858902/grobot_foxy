################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(grobot_node)

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
find_package(PCL REQUIRED)

find_package(ament_index_cpp REQUIRED)

## EXPORTS
include_directories(include)
include_directories(${ament_INCLUDE_DIRS})



################################################################################
# Build
################################################################################
add_executable(lidar_calibration src/lidar_calibration.cpp)  
ament_target_dependencies(lidar_calibration rclcpp std_srvs  sensor_msgs  tf2 tf2_ros tf2_sensor_msgs gazebo_msgs tf2_geometry_msgs std_msgs geometry_msgs nav2_msgs ament_index_cpp yaml-cpp PCL)


################################################################################
# Install
################################################################################
install(
  DIRECTORY launch include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lidar_calibration
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# Macro for ament package
################################################################################
ament_package()
