"""Path source; simulation launch starts only an inactive placeholder."""

from robot_path.settings import path_settings
import shlex
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_server", default_value=str(path_settings()["path_planner"]["start_server"]).lower()),
            DeclareLaunchArgument(
                "gripper",
                default_value=str(path_settings()["path"]["gripper"]),
                choices=["pneumatic", "msg_ai_100"],
            ),
            DeclareLaunchArgument("width", default_value=str(path_settings()["path"]["width"])),
            DeclareLaunchArgument("distance", default_value=str(path_settings()["path"]["distance"])),
            DeclareLaunchArgument("height", default_value=str(path_settings()["path"]["height"])),
            DeclareLaunchArgument("speed", default_value=str(path_settings()["path"]["speed"]),
                                  description="Cartesian path rate multiplier"),
            DeclareLaunchArgument("tip", default_value=str(path_settings()["path"]["tip"])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("robot_server"), "launch", "server.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_server")),
                launch_arguments={
                    "backend": "simulation",
                }.items(),
            ),
            Node(
                package="robot_path",
                executable="path_node.py",
                prefix=[shlex.quote(sys.executable)],
                output="screen",
                emulate_tty=True,
                arguments=[
                    "--gripper",
                    LaunchConfiguration("gripper"),
                    "--width",
                    LaunchConfiguration("width"),
                    "--distance",
                    LaunchConfiguration("distance"),
                    "--height",
                    LaunchConfiguration("height"),
                    "--speed",
                    LaunchConfiguration("speed"),
                    "--tip",
                    LaunchConfiguration("tip"),
                    "--autostart",
                    LaunchConfiguration("start_server"),
                ],
            ),
        ]
    )
