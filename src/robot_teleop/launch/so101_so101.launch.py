from pathlib import Path
import shlex
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from robot_teleop.config import teleop_settings, tracking_profile


def follower_visualization(context):
    follower = tracking_profile(LaunchConfiguration('profile').perform(context))['follower']
    launch_file = PathJoinSubstitution([FindPackageShare('robot_server'), 'launch', 'visualization.launch.py'])
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file),
                                     launch_arguments={**follower, 'foxglove': 'false'}.items())]


def generate_launch_description():
    hardware = teleop_settings()['hardware']
    calibration = Path(hardware['calibration_dir']).expanduser() / (hardware['leader_id'] + '.json')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=hardware['leader_port']),
        DeclareLaunchArgument('calibration', default_value=str(calibration)),
        DeclareLaunchArgument('profile', default_value=teleop_settings()['launch']['default_profile'], choices=list(teleop_settings()['teleop'])),
        DeclareLaunchArgument('visualize', default_value=str(teleop_settings()['launch']['visualize']).lower(),
                              description='Publish follower model and TF. Disable when standalone viz is running.'),
        GroupAction(condition=IfCondition(LaunchConfiguration('visualize')),
                    forwarding=False, launch_configurations={'profile': LaunchConfiguration('profile')},
                    actions=[OpaqueFunction(function=follower_visualization)]),
        Node(package='robot_teleop', executable='so101.py',
             prefix=[shlex.quote(sys.executable)], output='screen', arguments=[
                 '--port', LaunchConfiguration('port'),
                 '--calibration', LaunchConfiguration('calibration'),
                 '--profile', LaunchConfiguration('profile'),
             ]),
    ])
