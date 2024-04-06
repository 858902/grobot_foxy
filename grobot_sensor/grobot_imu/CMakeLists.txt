cmake_minimum_required(VERSION 3.5)
project(grobot_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  "${PROJECT_SOURCE_DIR}/include/mw"
  "${PROJECT_SOURCE_DIR}/include/serial"
)

add_executable(${PROJECT_NAME}_node src/listener.cpp serial/list_ports_linux.cc serial/serial.cc serial/unix.cc mw/mw_ahrs.cpp)
ament_target_dependencies(grobot_imu_node
"rclcpp"
"std_msgs"
"sensor_msgs"
"geometry_msgs"
"tf2"
"tf2_ros"
"tf2_msgs"
)

find_library(MW_AHRS_amd64_LIBRARY
    NAMES MW_AHRS_amd64.a\\
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

# file(TO_CMAKE_PATH "/feature_ws/src/grobot_foxy/grobot_sensor/grobot_imu/lib/MW_AHRS_amd64.a" ABSOLUTE_LIB_PATH)
 target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_AHRS_amd64.a)
# target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_AHRS_aarch64.a)
# target_link_libraries(${PROJECT_NAME}_node
#     ${ABSOLUTE_LIB_PATH}
# )
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)
ament_package()
