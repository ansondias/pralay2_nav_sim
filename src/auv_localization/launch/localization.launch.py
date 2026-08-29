from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ekf_config = PathJoinSubstitution(
        [FindPackageShare("auv_localization"), "config", "ekf.yaml"]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    # No GPS/DVL-based global localization in this sandbox: map == odom.
    # Replace this with amcl or your real global localization node when
    # you're ready to test that part too.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    return LaunchDescription([ekf_node, map_to_odom])
