from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_server"), "config", "so101_rgb3.yaml"]
                ),
            ),
            DeclareLaunchArgument("mode", default_value="realtime"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("model", default_value="false"),
            DeclareLaunchArgument(
                "model_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("model_client"), "config", "act.yaml"]
                ),
            ),
            DeclareLaunchArgument("model_url", default_value="ws://127.0.0.1:8000"),
            DeclareLaunchArgument(
                "instruction", default_value="Move the cube to the target"
            ),
            include(
                "robot_server",
                "server.launch.py",
                {
                    "deployment": LaunchConfiguration("config"),
                    "mode": LaunchConfiguration("mode"),
                    "foxglove": LaunchConfiguration("foxglove"),
                },
            ),
            include(
                "model_client",
                "client.launch.py",
                {
                    "config": LaunchConfiguration("model_config"),
                    "model_url": LaunchConfiguration("model_url"),
                    "instruction": LaunchConfiguration("instruction"),
                },
                IfCondition(LaunchConfiguration("model")),
            ),
        ]
    )
