import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Which hull to put in the water. Both expose the same topics/frames
    # (/cmd_vel, /dvl_odom, /imu/data, base_link) so the bridge, EKF and
    # Nav2 don't care; "pralay2" additionally runs ros2_control fins and a
    # propeller through gz_ros2_control (see ros_pralay_2_v1's .gazebo.xacro).
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="pralay2",
        choices=["pralay2", "auv"],
        description='"pralay2": the real ROS_PRALAY_2_V1 export (ros_pralay_2_v1); '
        '"auv": the generic placeholder torpedo (auv_description).',
    )
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

    # sdformat turns the URDF's package://ros_pralay_2_v1/meshes/*.STL into
    # model://ros_pralay_2_v1/meshes/*.STL, which gz-sim resolves by looking
    # for a "ros_pralay_2_v1" directory under each GZ_SIM_RESOURCE_PATH
    # entry. ros_pralay_2_v1's environment hook adds <prefix>/share when the
    # workspace is sourced, but a terminal sourced before that package was
    # first built doesn't have it (symptom: "Unable to find file with URI
    # [model://ros_pralay_2_v1/meshes/...]" and an invisible robot). Set it
    # here too so the launch works regardless of the shell's state.
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.dirname(get_package_share_directory("ros_pralay_2_v1")),
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

    robot = LaunchConfiguration("robot")
    is_pralay2 = PythonExpression(["'", robot, "' == 'pralay2'"])

    placeholder_xacro = PathJoinSubstitution(
        [FindPackageShare("auv_description"), "urdf", "auv.urdf.xacro"]
    )
    pralay2_xacro = PathJoinSubstitution(
        [FindPackageShare("ros_pralay_2_v1"), "urdf", "ROS_PRALAY_2_V1.urdf.xacro"]
    )
    # sim_mode:=true swaps the Pralay-2's ros2_control hardware for
    # gz_ros2_control and pulls in its Gazebo plugins; the placeholder has
    # no such switch, its gazebo xacro is always included.
    xacro_cmd = PythonExpression(
        [
            "'xacro ", pralay2_xacro, " sim_mode:=true' if ", is_pralay2,
            " else 'xacro ", placeholder_xacro, "'",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
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
            # Model name doubles as the OceanCurrent override topic prefix:
            # /model/<name>/ocean_current (auv_gazebo/README.md).
            "-name", robot,
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
            gz_resource_path,
            robot_arg,
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
