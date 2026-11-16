"""Dive to depth, hold it, then run to goals clicked in RViz.

One command for the whole thing:

    ros2 launch auv_bringup mission.launch.py

then click "2D Goal Pose" in RViz. The goal is held until the hull has
settled at depth, and released to Nav2 only then; click it during the dive
and it is queued, not lost.

It starts, in order:
  * the Gazebo world, the hull, the EKF, the map and Nav2 (sim.launch.py),
    with the hull spawned at z=4.5, i.e. 0.5 m down, so the 1 m depth
    target below is a descent you can watch;
  * auv_control's depth_hold (plus the simulated Bar30 in sim);
  * auv_bringup's dive_and_navigate, which hands depth_hold the target,
    waits until the depth has been inside the 0.8..1.2 m band for
    settle_time, and only then releases goals to Nav2. Goals arrive on
    goal_topic (/mission/goal_pose), which is where this package's RViz
    config points its "2D Goal Pose" tool -- deliberately not Nav2's own
    /goal_pose, which bt_navigator acts on immediately.

Depth hold keeps running through the transit: it owns the elevator (left
and right) fins, Nav2 only ever commands surge and yaw, so they do not
fight. Any depth excursion during the transit is logged and counted.

The plan itself is 2-D. For RViz, plan_at_depth republishes /plan and
/local_plan as /plan_at_depth and /local_plan_at_depth with z = -target
depth, so with the 3-D EKF the path is drawn where the hull will fly it
rather than on the surface.

Useful arguments:
    goal_topic                 where goals are read from (RViz tool topic)
    target_depth               metres below the surface (default 1.0)
    settle_time                seconds in band before the transit starts
    z                          spawn height (surface is z=5, so 4.5 = 0.5 m down)
    bringup:=false             Nav2/Gazebo already running elsewhere
    sim:=false                 real vehicle (real Bar30, no /depth_setpoint)
    headless:=true rviz:=false lighter run on a small Docker VM
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("bringup", default_value="true"),
        DeclareLaunchArgument("sim", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("z", default_value="4.5"),
        DeclareLaunchArgument("target_depth", default_value="1.0"),
        DeclareLaunchArgument("settle_time", default_value="5.0"),
        DeclareLaunchArgument("goal_topic", default_value="/mission/goal_pose"),
        DeclareLaunchArgument("enable_delivery", default_value="false"),
        DeclareLaunchArgument("cruise_speed", default_value="1.0"),
        DeclareLaunchArgument("on_abort", default_value="hold"),
    ]

    sim = LaunchConfiguration("sim")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("auv_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "z": LaunchConfiguration("z"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("bringup")),
    )

    depth_hold = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("auv_control"), "launch", "depth_hold.launch.py")
        ),
        launch_arguments={
            "sim": sim,
            "target_depth": LaunchConfiguration("target_depth"),
            "enabled": "false",   # phase 1 DEPLOY is inert; phase 4 ARM enables it
        }.items(),
    )

    # Held back a few seconds so it does not start counting its 30 s
    # "is the depth controller alive?" timeout while Gazebo is still loading.
    mission = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="auv_bringup",
                executable="dive_and_navigate",
                name="dive_and_navigate",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(sim, value_type=bool),
                        "target_depth": ParameterValue(
                            LaunchConfiguration("target_depth"), value_type=float),
                        "settle_time": ParameterValue(
                            LaunchConfiguration("settle_time"), value_type=float),
                        "goal_topic": ParameterValue(
                            LaunchConfiguration("goal_topic"), value_type=str),
                        "enable_delivery": ParameterValue(
                            LaunchConfiguration("enable_delivery"), value_type=bool),
                        "cruise_speed": ParameterValue(
                            LaunchConfiguration("cruise_speed"), value_type=float),
                        "on_abort": ParameterValue(
                            LaunchConfiguration("on_abort"), value_type=str),
                        # Must match the planner's current model and the thrust cap.
                        "water_current_speed": 0.3,
                        "water_current_direction_north": 135.0,
                        "max_attainable_speed": 2.06,
                    }
                ],
            )
        ],
    )

    # RViz-only: Nav2's /plan and /local_plan have z = 0 (the surface);
    # republish them at the held depth so they sit on the hull in the 3-D
    # view. Follows the sequencer's latched /depth_hold/target_depth.
    plan_at_depth = Node(
        package="auv_bringup",
        executable="plan_at_depth.py",
        name="plan_at_depth",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(sim, value_type=bool),
                "default_depth": ParameterValue(
                    LaunchConfiguration("target_depth"), value_type=float),
            }
        ],
    )

    return LaunchDescription(args + [bringup, depth_hold, mission, plan_at_depth])
