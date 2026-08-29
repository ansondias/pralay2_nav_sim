import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("auv_gazebo"), "worlds", "ocean_arena.sdf"]
        ),
        description="Path to the Gazebo (gz-sim) world SDF file.",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the gz-sim server only (-s), no GUI client.",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    # ocean_arena's seafloor plane is at z=0 with obstacles built above it
    # (e.g. reef tops around z=2) -- spawn well clear of both, not below
    # the floor (a negative default here previously spawned the hull
    # underground, deeply interpenetrating the floor plane and blowing up
    # the physics solver on the first few steps).
    z_arg = DeclareLaunchArgument("z", default_value="3.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    gz_sim_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r"]}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r -s"]}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("auv_description"), "urdf", "auv.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_auv",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "auv",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    bridge_config = PathJoinSubstitution(
        [FindPackageShare("auv_gazebo"), "config", "bridge.yaml"]
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            gz_sim_with_gui,
            gz_sim_headless,
            robot_state_publisher,
            spawn,
            bridge,
        ]
    )
