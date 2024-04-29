cmake_minimum_required(VERSION 3.5)
project(grobot_perception)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(grobot_sensor_fusion_node src/grobot_sensor_fusion_node.cpp)  # 파일 이름 수정
ament_target_dependencies(grobot_sensor_fusion_node rclcpp sensor_msgs pcl_conversions)

target_link_libraries(grobot_sensor_fusion_node
  ${tf2_ros_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  grobot_sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
