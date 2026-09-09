cmake_minimum_required(VERSION 3.8)

project(ros_pralay_2_v1)


find_package(ament_cmake REQUIRED)


install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

# Puts <prefix>/share on GZ_SIM_RESOURCE_PATH when the workspace is sourced,
# so gz-sim can resolve the package://ros_pralay_2_v1/meshes/*.STL URIs in
# the URDF (RViz resolves them through ament on its own).
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")


ament_package()
