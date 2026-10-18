import shlex
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bundle'),DeclareLaunchArgument('url',default_value='ws://127.0.0.1:8000'),
        DeclareLaunchArgument('foxglove',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_server'),'launch','server.launch.py'])),
            launch_arguments={'foxglove':LaunchConfiguration('foxglove')}.items()),
        Node(package='model_client',executable='model_client',prefix=[shlex.quote(sys.executable)],output='screen',
             arguments=['--bundle',LaunchConfiguration('bundle'),'--url',LaunchConfiguration('url')]),
    ])
