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
    headless_arg = DeclareLaunchArgument("headless", default_value="false")
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
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
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

    # 4. Default nav2 navigation stack, GridBased planner = auv_planning::AStarPlanner
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
            headless_arg,
            rviz_arg,
            map_arg,
            params_arg,
            spawn,
            localization,
            map_server,
            map_server_lifecycle,
            navigation,
            rviz,
        ]
    )
