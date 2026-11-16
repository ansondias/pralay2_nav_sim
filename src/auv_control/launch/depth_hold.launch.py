"""Depth hold for the Pralay-2 (dive to target_depth and keep it).

    ros2 launch auv_control depth_hold.launch.py                  # real AUV, config/depth_hold.yaml
    ros2 launch auv_control depth_hold.launch.py sim:=true        # under Gazebo
    ros2 launch auv_control depth_hold.launch.py target_depth:=2.0

Publishes the elevator angle on /depth_hold/elevator; cmd_vel_to_actuators
mixes it into the four fins (rudder still from cmd_vel). Re-target live:
    ros2 topic pub --once /depth_hold/target_depth std_msgs/msg/Float64 "{data: 1.5}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("auv_control"), "config", "depth_hold.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false"),
            DeclareLaunchArgument("target_depth", default_value="1.0"),
            # The mission sequencer starts it disabled (phase 1 DEPLOY is inert)
            # and arms it in phase 4; standalone use keeps the old behaviour.
            DeclareLaunchArgument("enabled", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=config),
            # Gazebo has no pressure sensor: sim_depth_sensor turns the
            # simulated odometry's Z into a Bar30 reading so the controller
            # closes its loop on real feedback instead of sitting in its
            # "no depth yet" state. Not started on the AUV, where the real
            # Bar30 driver publishes these topics.
            Node(
                package="auv_control",
                executable="sim_depth_sensor",
                name="sim_depth_sensor",
                output="screen",
                parameters=[{"use_sim_time": True, "surface_z": 5.0}],
                condition=IfCondition(LaunchConfiguration("sim")),
            ),
            Node(
                package="auv_control",
                executable="depth_hold",
                name="depth_hold",
                output="screen",
                # The node is C++ now, so it rejects a parameter whose type
                # doesn't match what it declared: launch arguments arrive as
                # strings and have to be typed explicitly here.
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "sim_mode": ParameterValue(
                            LaunchConfiguration("sim"), value_type=bool),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("sim"), value_type=bool),
                        "target_depth": ParameterValue(
                            LaunchConfiguration("target_depth"), value_type=float),
                        "enabled": ParameterValue(
                            LaunchConfiguration("enabled"), value_type=bool),
                        # In sim the stand-in sensor gives depth in metres
                        # directly; the pressure path (and its surface
                        # auto-zero, which assumes the AUV starts at the
                        # surface) is the hardware one.
                        "depth_source": ParameterValue(
                            PythonExpression(
                                ["'depth' if '", LaunchConfiguration("sim"),
                                 "' == 'true' else 'pressure'"]),
                            value_type=str),
                    },
                ],
            ),
        ]
    )
