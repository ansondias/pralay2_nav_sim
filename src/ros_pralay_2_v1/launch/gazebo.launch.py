"""Pralay-2 in the ocean_arena Gazebo world, with RViz, no Nav2.

Physics-level bring-up for checking the model itself: hull floats level,
fins and propeller respond to /cmd_vel, /dvl_odom and /imu/data come out.
Everything is delegated to the packages that already own it:

  * auv_gazebo/launch/spawn.launch.py  (robot:=pralay2) - gz-sim + world,
    robot_state_publisher with sim_mode:=true, spawn, ros_gz bridge.
  * auv_control/launch/controllers.launch.py (sim:=true) - spawns the
    joint_state_broadcaster / fin_controller / thruster_controller into the
    controller_manager that gz_ros2_control runs inside gz-sim, plus the
    cmd_vel -> fin/propeller command mapper.

For the full navigation stack use `ros2 launch auv_bringup sim.launch.py
robot:=pralay2` instead; it includes the same two pieces.

Try:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        "{linear: {x: 1.0}, angular: {z: 0.2}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("auv_gazebo"), "worlds", "ocean_arena.sdf"]
        ),
    )
    headless_arg = DeclareLaunchArgument("headless", default_value="false")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="3.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_gazebo"), "launch", "spawn.launch.py"
            )
        ),
        launch_arguments={
            "robot": "pralay2",
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_control"),
                "launch",
                "controllers.launch.py",
            )
        ),
        launch_arguments={"sim": "true", "use_sim_time": "true"}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ros_pralay_2_v1"), "rviz", "pralay2.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            rviz_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            spawn,
            controllers,
            rviz,
        ]
    )
