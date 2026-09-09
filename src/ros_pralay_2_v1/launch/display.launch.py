"""RViz-only view of the Pralay-2 model (no simulator, no hardware).

Processes the xacro with sim_mode:=false, so what you see is exactly the
SolidWorks export (mesh collisions and all). joint_state_publisher_gui gives
sliders for the four fins and the propeller; turn it off with gui:=false if
something else (ros2_control, Gazebo) is already publishing /joint_states.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start joint_state_publisher_gui (fin/propeller sliders).",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ros_pralay_2_v1"), "rviz", "pralay2.rviz"]
        ),
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ros_pralay_2_v1"), "urdf", "ROS_PRALAY_2_V1.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file, " sim_mode:=false use_ros2_control:=false"]),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            gui_arg,
            rviz_config_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
