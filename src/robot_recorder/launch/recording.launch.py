from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default = str(
        Path(get_package_share_directory("robot_recorder")) / "config/recording.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default),
            Node(
                package="robot_recorder",
                executable="recorder_node",
                name="robot_recorder",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
