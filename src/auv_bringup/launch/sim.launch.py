import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="pralay2",
        choices=["pralay2", "auv"],
        description='Hull to simulate: "pralay2" (real ROS_PRALAY_2_V1 export, '
        'with ros2_control fins/propeller) or "auv" (generic placeholder).',
    )
    headless_arg = DeclareLaunchArgument("headless", default_value="false")
    # Spawn pose, passed through to auv_gazebo/spawn.launch.py. z is a world
    # height, not a depth: the free surface is at z = 5 (surface_z in the
    # hull's gazebo xacro), so z:=4.5 spawns the hull 0.5 m down and a 1 m
    # depth target is then a genuine descent.
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="3.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [FindPackageShare("auv_bringup"), "maps", "ocean_arena.yaml"]
        ),
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("auv_navigation"), "config", "nav2_params.yaml"]
        ),
    )

    # 1. Gazebo world + robot spawn + ros_gz bridge (cmd_vel, dvl_odom, imu, clock)
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_gazebo"), "launch", "spawn.launch.py"
            )
        ),
        launch_arguments={
            "robot": LaunchConfiguration("robot"),
            "headless": LaunchConfiguration("headless"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    # 1b. Pralay-2 only: ros2_control spawners (joint_state_broadcaster,
    #     fin_controller, thruster_controller) into the controller_manager
    #     that gz_ros2_control runs inside gz-sim, plus the cmd_vel ->
    #     fin/propeller mapper. Nav2's cmd_vel then both moves the hull
    #     (ThrustControl) and animates the actuators.
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_control"),
                "launch",
                "controllers.launch.py",
            )
        ),
        launch_arguments={"sim": "true", "use_sim_time": "true"}.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("robot"), "' == 'pralay2'"])
        ),
    )

    # 2. Sensor fusion: EKF(imu + dvl-like odom) -> odom->base_link tf,
    #    plus a fixed map->odom (no global localization node in this sandbox).
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_localization"),
                "launch",
                "localization.launch.py",
            )
        )
    )

    # 3. Static obstacle map, feeding the global costmap's static_layer.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": LaunchConfiguration("map")}],
    )
    map_server_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    # 4. Keeps collision_monitor's required-but-unused observation source
    #    "alive": a source that never publishes anything gets treated as
    #    invalid, and collision_monitor's fallback for that is to zero the
    #    robot's outgoing cmd_vel -- silently blocking all Nav2-driven
    #    motion (manual cmd_vel still worked, since that bypasses it).
    unused_scan_publisher = Node(
        package="auv_bringup",
        executable="unused_scan_publisher.py",
        name="unused_scan_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 5. Default nav2 navigation stack, GridBased planner = auv_planning::AStarPlanner
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_navigation"),
                "launch",
                "navigation.launch.py",
            )
        ),
        launch_arguments={"params_file": LaunchConfiguration("params_file")}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("auv_bringup"), "rviz", "nav2_default_view.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            robot_arg,
            headless_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            rviz_arg,
            map_arg,
            params_arg,
            spawn,
            controllers,
            localization,
            map_server,
            map_server_lifecycle,
            unused_scan_publisher,
            navigation,
            rviz,
        ]
    )
