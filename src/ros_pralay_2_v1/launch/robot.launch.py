import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)

from launch_ros.parameter_descriptions import ParameterValue

from dotenv import load_dotenv

# Same .env / JETSON_FILES_DIR that mission_handler.launch.py and
# jetson_tcp_node_d_m.py already read - load_dotenv() with no path
# searches upward from the CWD, so this finds the repo root .env when
# launched from there (true both for `ros2 launch` run by hand from the
# repo root, and for the auv-*.service systemd units, which all set
# WorkingDirectory=<repo root>).
load_dotenv()


def generate_launch_description():
     # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Check if we are using ros2_control
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # ros2_control.urdf.xacro's crash_log_dir arg - one place (.env) to
    # change this per machine instead of hardcoding it in the xacro.
    jetson_files_dir = os.getenv('JETSON_FILES_DIR',os.path.join(os.path.dirname(__file__), 'jetson_files'))
    crash_log_dir = os.path.join(jetson_files_dir, 'controllers_crashes')




    loop_rate = os.getenv('LOOP_RATE', "100")
    budrate = os.getenv('BUDRATE', "1000000")
    timeout = os.getenv('TIMEOUT_MS', "50")
    device_id_master = os.getenv('DEVICE_ID_MASTER', '/dev/stm32_1')
    device_id_imu = os.getenv('DEVICE_ID_IMU', '/dev/stm32_3')
    device_id_actuators = os.getenv('DEVICE_ID_ACTUATORS', '/dev/stm32_2')
    device_id_sensors = os.getenv('DEVICE_ID_SENSORS', '/dev/stm32_1')
    
    
    
    
    # -----------------------------
    # Robot URDF
    # -----------------------------
    robot_description_file = PathJoinSubstitution(
    [
        FindPackageShare("ros_pralay_2_v1"),
        "urdf",
        "ROS_PRALAY_2_V1.urdf.xacro"
    ]
)


    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                robot_description_file,
                ' use_ros2_control:=', use_ros2_control,
                ' crash_log_dir:=', crash_log_dir,
                ' loop_rate:=', loop_rate,
                ' budrate:=', budrate,
                ' timeout:=', timeout,
                ' device_id_master:=', device_id_master,
                ' device_id_imu:=', device_id_imu,
                ' device_id_actuators:=', device_id_actuators,
                ' device_id_sensors:=', device_id_sensors,

            ]
        ),
        value_type=str
    )





    return LaunchDescription([


        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        # -----------------------------
        # Robot State Publisher
        # -----------------------------

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",

            parameters=[
                {
                    "robot_description": robot_description,
                    'use_sim_time': use_sim_time,
                }
            ],

            output="screen"
        ),





    ])