import shlex
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot', default_value='so101'),
        DeclareLaunchArgument('paced', default_value='false'), DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_port', default_value='8766'), DeclareLaunchArgument('recordings', default_value='recordings'),
        Node(package='robot_server', executable='server', prefix=[shlex.quote(sys.executable)], output='screen',
             parameters=[{'robot': LaunchConfiguration('robot'), 'paced': ParameterValue(LaunchConfiguration('paced'), value_type=bool),
                          'recordings': LaunchConfiguration('recordings')}]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('robot_server'), 'launch', 'visualization.launch.py'])),
            launch_arguments={'robot': LaunchConfiguration('robot'), 'foxglove': LaunchConfiguration('foxglove'),
                              'port': LaunchConfiguration('foxglove_port')}.items())])])
