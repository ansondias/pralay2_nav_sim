cmake_minimum_required(VERSION 3.8)
project(auv_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Bar30 depth -> PoseWithCovarianceStamped for the EKF's pose0 input.
install(PROGRAMS
  scripts/depth_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
