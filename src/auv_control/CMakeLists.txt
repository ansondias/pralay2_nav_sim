cmake_minimum_required(VERSION 3.8)
project(auv_control)

# Build type changed from ament_python to ament_cmake when the depth-hold
# controller was ported to C++ (src/depth_hold.cpp). ament_cmake_python still
# installs the auv_control Python module and its nodes, exactly as before --
# same executable names, so no launch file had to change. This is the same
# mixed C++/Python layout auv_planning already uses. setup.py / setup.cfg are
# no longer consulted by the build; the executables below are the truth.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- C++ nodes ---------------------------------------------------------------
add_executable(depth_hold src/depth_hold.cpp)
ament_target_dependencies(depth_hold
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS depth_hold
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Python module + nodes ----------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# Installed under the names the launch files already use. depth_hold.py keeps
# its own name (depth_hold_py): it is the original prototype of the C++ node
# above, kept runnable for comparison, not started by any launch file.
install(PROGRAMS auv_control/cmd_vel_to_actuators.py
  DESTINATION lib/${PROJECT_NAME} RENAME cmd_vel_to_actuators)
install(PROGRAMS auv_control/fins_control.py
  DESTINATION lib/${PROJECT_NAME} RENAME fins_control)
install(PROGRAMS auv_control/depth_hold.py
  DESTINATION lib/${PROJECT_NAME} RENAME depth_hold_py)
install(PROGRAMS auv_control/sim_depth_sensor.py
  DESTINATION lib/${PROJECT_NAME} RENAME sim_depth_sensor)

# ---- launch / config ------------------------------------------------------------
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
