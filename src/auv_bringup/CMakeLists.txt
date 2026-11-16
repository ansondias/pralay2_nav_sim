cmake_minimum_required(VERSION 3.8)
project(auv_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Mission sequencer: dive to depth, hold it, then run to the target.
add_executable(dive_and_navigate src/dive_and_navigate.cpp)
ament_target_dependencies(dive_and_navigate
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS dive_and_navigate DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch maps rviz scripts config
  DESTINATION share/${PROJECT_NAME}
)

# Point Fast DDS at config/fastdds_no_shm.xml (UDP only, no shared-memory
# transport) whenever the workspace is sourced; see that file for why.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/fastdds_profile.dsv.in")

install(PROGRAMS
  scripts/unused_scan_publisher.py
  scripts/plan_at_depth.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
