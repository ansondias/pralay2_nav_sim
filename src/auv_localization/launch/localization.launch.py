"""3-D state estimate for the Pralay-2: odom->base_link from the EKF.

    ros2 launch auv_localization localization.launch.py                      # Gazebo (default)
    ros2 launch auv_localization localization.launch.py use_sim_time:=false  # real AUV

Inputs (config/ekf.yaml): /dvl_odom (body velocities), /imu/data (attitude
and rates) and /bar30/depth, which depth_to_pose turns into the /depth/pose
z-only pose the EKF fuses. map->odom is a fixed identity here (no global
localization node in this sandbox), so map z = 0 is the free surface and
the hull sits below it in RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution(
        [FindPackageShare("auv_localization"), "config", "ekf.yaml"]
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    )

    depth_to_pose = Node(
        package="auv_localization",
        executable="depth_to_pose.py",
        name="depth_to_pose",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "depth_topic": LaunchConfiguration("depth_topic"),
        }],
    )

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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "depth_topic", default_value="/bar30/depth",
            description="Float64 depth in metres below the surface, fused as the EKF's z.",
        ),
        ekf_node,
        depth_to_pose,
        map_to_odom,
    ])
