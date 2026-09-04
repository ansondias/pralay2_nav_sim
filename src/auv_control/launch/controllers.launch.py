from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    # -----------------------------
    # Controllers YAML
    # -----------------------------
    controllers_yaml = PathJoinSubstitution([FindPackageShare("auv_control"),"config","controllers.yaml"])

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='false',description='Set "true" for Isaac Sim (enables sim time/mocks), "false" for physical AUV hardware.')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_val = {'use_sim_time': use_sim_time}


    return LaunchDescription([
        use_sim_time_arg,
        # -----------------------------
        # ros2_control Controller Manager
        # -----------------------------
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                controllers_yaml,
                use_sim_time_val
                ],
            output="screen",
            remappings=[
                (
                    "~/robot_description",
                    "/robot_description"
                )
            ]
        ),




        # -----------------------------
        # Spawn joint state broadcaster
        # -----------------------------
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        "/controller_manager"
                    ],
                    parameters=[use_sim_time_val],
                    output="screen"
                )
            ]
        ),




        # -----------------------------
        # Spawn fin controller
        # -----------------------------
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "fin_controller",
                        "--controller-manager",
                        "/controller_manager"
                    ],
                    parameters=[use_sim_time_val],
                    output="screen"
                )
            ]
        ),




        # -----------------------------
        # Spawn thruster controller
        # -----------------------------
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "thruster_controller",
                        "--controller-manager",
                        "/controller_manager"
                    ],
                    parameters=[use_sim_time_val],
                    output="screen"
                )
            ]
        ),



    ])