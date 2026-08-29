cmake_minimum_required(VERSION 3.8)
project(auv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch maps rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/unused_scan_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
