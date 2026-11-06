"""Standalone Foxglove bridge, including PAROL6's uppercase STL assets."""

from robot_core.config import robot_settings
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=str(robot_settings()["visualization"]["port"])),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            parameters=[{
                "address": robot_settings()["visualization"]["host"],
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "asset_uri_allowlist": [
                    r"^package://robot_description/(?:[-\w]+/)*[-\w.]+\.(?:stl|STL|dae|obj|png|jpg)$"
                ],
            }],
        ),
    ])
