from pathlib import Path
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
    return LaunchDescription([DeclareLaunchArgument('port', description='Explicit serial port for the SO101 leader'), DeclareLaunchArgument('calibration',
                                                                                                                                            default_value=str(
                                                                                                                                            Path.home() / '.cache/huggingface/lerobot/calibration/teleoperators/so_leader/so101-leader.json')),
        DeclareLaunchArgument('keyboard', default_value='auto'), DeclareLaunchArgument('start_server', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'), DeclareLaunchArgument('foxglove_port', default_value='8766'),
        DeclareLaunchArgument('output', default_value='training/datasets'),
        DeclareLaunchArgument('ik_config', default_value='', description='Optional FK/IK TOML configuration'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('robot_server'), 'launch', 'server.launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_server')),
            launch_arguments={'foxglove': LaunchConfiguration('foxglove'), 'foxglove_port': LaunchConfiguration('foxglove_port'),
                              'recordings': LaunchConfiguration('output')}.items()),
        Node(package='robot_teleop', executable='so101', prefix=[shlex.quote(sys.executable)], output='screen',
             arguments=['--port', LaunchConfiguration('port'), '--calibration', LaunchConfiguration('calibration'), '--keyboard',
                        LaunchConfiguration('keyboard'), '--ik-config', LaunchConfiguration('ik_config')]), ])
