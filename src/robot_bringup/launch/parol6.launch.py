"""Composition defaults; every launch choice remains overridable by the operator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_server"), "config", "parol6.yaml"]
                ),
                description="Robot/backend/camera configuration owned by Robot Server",
            ),
            DeclareLaunchArgument(
                "model_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("model_client"), "config", "act.yaml"]
                ),
                description="Input features for the optional Model Client",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robot_bringup"),
                            "launch",
                            "system.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config": LaunchConfiguration("config"),
                    "model_config": LaunchConfiguration("model_config"),
                }.items(),
            ),
        ]
    )
