cmake_minimum_required(VERSION 3.8)

project(ros_pralay_2_v1)


find_package(ament_cmake REQUIRED)


install(
  DIRECTORY
    urdf
    meshes
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()