import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("auv_navigation"), "config", "nav2_params.yaml"]
        ),
        description="Nav2 params file (planner_server/controller_server/etc.).",
    )
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    # bt_navigator needs an absolute path to the custom (no-Spin) behavior
    # tree; the yaml carries just the file name, resolved here at launch.
    bt_xml = os.path.join(
        get_package_share_directory("auv_navigation"),
        "behavior_trees",
        "navigate_to_pose_auv.xml",
    )
    params_with_bt = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={"default_nav_to_pose_bt_xml": bt_xml},
        convert_types=True,
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": params_with_bt,
            "autostart": LaunchConfiguration("autostart"),
        }.items(),
    )

    return LaunchDescription([params_arg, autostart_arg, navigation_launch])
