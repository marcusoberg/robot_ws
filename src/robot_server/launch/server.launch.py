"""The robot owner and optional visualization bridge; startup never enables motion."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("robot_server"))
    bridge = Path(get_package_share_directory("foxglove_bridge"))
    return LaunchDescription([
        DeclareLaunchArgument("deployment", default_value=str(share / "config/so101_rgb3.yaml")),
        DeclareLaunchArgument("backend", default_value="mujoco", choices=["mujoco", "serial"]),
        DeclareLaunchArgument("mode", default_value="realtime", choices=["realtime", "stepped"]),
        DeclareLaunchArgument("action_period", default_value="0.05"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8766"),
        Node(package="robot_server", executable="server", name="robot_server", output="screen", parameters=[{
            "deployment": LaunchConfiguration("deployment"),
            "backend": LaunchConfiguration("backend"),
            "mode": LaunchConfiguration("mode"),
            "action_period": ParameterValue(LaunchConfiguration("action_period"), value_type=float),
        }]),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(bridge / "launch/foxglove_bridge_launch.xml")),
            condition=IfCondition(LaunchConfiguration("foxglove")),
            launch_arguments={"address": "127.0.0.1", "port": LaunchConfiguration("foxglove_port")}.items(),
        ),
    ])
