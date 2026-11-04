from pathlib import Path
import shlex
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration = Path.home() / '.cache/huggingface/lerobot/calibration/teleoperators/so_leader/so101-leader.json'
    return LaunchDescription([
        DeclareLaunchArgument('port', description='Serial port for the SO101 leader'),
        DeclareLaunchArgument('calibration', default_value=str(calibration)),
        DeclareLaunchArgument('ik_config', default_value=PathJoinSubstitution([
            FindPackageShare('robot_teleop'), 'config', 'so101_ik.toml',
        ])),
        Node(package='robot_teleop', executable='so101.py',
             prefix=[shlex.quote(sys.executable)], output='screen', arguments=[
                 '--port', LaunchConfiguration('port'),
                 '--calibration', LaunchConfiguration('calibration'),
                 '--ik-config', LaunchConfiguration('ik_config'),
             ]),
    ])
