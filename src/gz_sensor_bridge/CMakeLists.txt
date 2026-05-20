cmake_minimum_required(VERSION 3.8)
project(gz_sensor_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(gz_camera_bridge_node src/gz_camera_bridge_node.cpp)
ament_target_dependencies(gz_camera_bridge_node
  rclcpp
  sensor_msgs
)
target_link_libraries(gz_camera_bridge_node
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)
target_compile_features(gz_camera_bridge_node PUBLIC cxx_std_17)

add_executable(gz_lidar_bridge_node src/gz_lidar_bridge_node.cpp)
ament_target_dependencies(gz_lidar_bridge_node
  rclcpp
  sensor_msgs
)
target_link_libraries(gz_lidar_bridge_node
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)
target_compile_features(gz_lidar_bridge_node PUBLIC cxx_std_17)

install(TARGETS
  gz_camera_bridge_node
  gz_lidar_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
