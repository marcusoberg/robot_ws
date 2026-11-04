"""PAROL6 path source with an optional simulator; the bridge is separate."""

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
            DeclareLaunchArgument("start_server", default_value="true"),
            DeclareLaunchArgument(
                "gripper",
                default_value="pneumatic",
                choices=["pneumatic", "msg_ai_100"],
            ),
            DeclareLaunchArgument("width", default_value="0.18"),
            DeclareLaunchArgument("distance", default_value="0.30"),
            DeclareLaunchArgument("height", default_value="0.16"),
            DeclareLaunchArgument("speed", default_value="1.0",
                                  description="Homing-rate multiplier: 0.5 is half speed"),
            DeclareLaunchArgument("tip", default_value="tool_tcp"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("robot_server"), "launch", "server.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_server")),
                launch_arguments={
                    "robot": "parol6",
                    "gripper": LaunchConfiguration("gripper"),
                    "paced": "true",
                    "foxglove": "false",
                    "scene": "robot_only",
                    "cameras": "false",
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
