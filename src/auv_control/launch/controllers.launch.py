"""ros2_control controllers for the Pralay-2 fins + propeller.

Two ways to run it:

  sim:=false (default) - real AUV / mock hardware. Starts a standalone
    ros2_control_node (controller_manager) from config/controllers.yaml,
    reading the URDF from /robot_description (robot.launch.py publishes it),
    then spawns the controllers into it.

  sim:=true - Gazebo. gz_ros2_control already runs a controller_manager
    *inside* gz-sim (loaded from ros_pralay_2_v1's .gazebo.xacro, with the
    same controllers.yaml), so no ros2_control_node is started here - a
    second one would claim the same /controller_manager name and the
    spawners would land in whichever answered first. Only the spawners run,
    plus cmd_vel_to_actuators so a /cmd_vel (teleop, Nav2) also moves the
    fins and spins the propeller.

Either way the resulting interface is identical:
  /fin_controller/commands       std_msgs/Float64MultiArray [Top, Left, Right, Bottom] rad
  /thruster_controller/commands  std_msgs/Float64MultiArray [T1]  (rad/s in sim, -1..1 duty on hardware)
  /joint_states                  from joint_state_broadcaster
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("auv_control"), "config", "controllers.yaml"]
    )

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description='"true" when running under Gazebo (gz_ros2_control hosts the '
        "controller_manager, so only spawners + the cmd_vel mapper start); "
        '"false" for the real AUV / mock hardware (starts ros2_control_node too).',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=LaunchConfiguration("sim"),
        description="Use /clock (defaults to the value of sim).",
    )
    actuator_bridge_arg = DeclareLaunchArgument(
        "cmd_vel_bridge",
        default_value=LaunchConfiguration("sim"),
        description="Start cmd_vel_to_actuators (cmd_vel -> fin/thruster commands). "
        "Defaults on in sim; on hardware the mission stack usually commands the "
        "controllers directly.",
    )
    sim = LaunchConfiguration("sim")
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    # Standalone controller manager: hardware/mock only.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml, use_sim_time],
        output="screen",
        remappings=[("~/robot_description", "/robot_description")],
        condition=UnlessCondition(sim),
    )

    def spawner(name, delay):
        return TimerAction(
            period=delay,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        name,
                        "--controller-manager",
                        "/controller_manager",
                        # gz-sim can take a while to load the plugin; keep
                        # trying instead of giving up after the default 10 s.
                        "--controller-manager-timeout",
                        "60",
                    ],
                    parameters=[use_sim_time],
                    output="screen",
                )
            ],
        )

    cmd_vel_bridge = Node(
        package="auv_control",
        executable="cmd_vel_to_actuators",
        name="cmd_vel_to_actuators",
        parameters=[use_sim_time],
        output="screen",
        condition=IfCondition(LaunchConfiguration("cmd_vel_bridge")),
    )

    return LaunchDescription(
        [
            sim_arg,
            use_sim_time_arg,
            actuator_bridge_arg,
            controller_manager,
            spawner("joint_state_broadcaster", 2.0),
            spawner("fin_controller", 2.0),
            spawner("thruster_controller", 4.0),
            cmd_vel_bridge,
        ]
    )
