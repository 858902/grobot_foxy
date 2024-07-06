cmake_minimum_required(VERSION 3.5)
project(grobot_perception)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(grobot_sensor_fusion_node src/grobot_sensor_fusion_node.cpp)
add_executable(laser_to_pointcloud src/grobot_laser2pcl2.cpp)
add_executable(sensor_fusion_node src/grobot_fusion_node.cpp)
add_executable(grobot_process_node src/grobot_process_node.cpp)

ament_target_dependencies(
  grobot_sensor_fusion_node
  rclcpp 
  sensor_msgs 
  pcl_conversions
  tf2_ros
  geometry_msgs
  laser_geometry 
  tf2_geometry_msgs
  )

ament_target_dependencies(
  laser_to_pointcloud
  rclcpp
  sensor_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
)

ament_target_dependencies(sensor_fusion_node
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
  tf2
  laser_geometry
  tf2_sensor_msgs
  PCL
)

ament_target_dependencies(grobot_process_node
  rclcpp
  sensor_msgs
  tf2_ros
  pcl_conversions
)


target_link_libraries(grobot_sensor_fusion_node
  ${tf2_ros_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sensor_fusion_node
  ${PCL_LIBRARIES}
)

target_link_libraries(grobot_process_node
  ${PCL_LIBRARIES}
)

target_link_libraries(laser_to_pointcloud
  ${tf2_ros_LIBRARIES}
)

install(TARGETS
  grobot_sensor_fusion_node
  laser_to_pointcloud
  sensor_fusion_node
  grobot_process_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()