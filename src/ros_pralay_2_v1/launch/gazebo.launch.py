from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():


    pkg = "ros_pralay_2_v1"


    urdf = PathJoinSubstitution(
        [
            FindPackageShare(pkg),
            "urdf",
            "ROS_PRALAY_2_V1.urdf"
        ]
    )


    return LaunchDescription([


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py"
                        ]
                    )
                ]
            ),
            launch_arguments={
                "gz_args": "-r empty.sdf"
            }.items()
        ),


        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "ros_pralay_2_v1",
                "-file",
                urdf
            ],
            output="screen"
        )


    ])