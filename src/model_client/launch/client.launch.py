from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=PathJoinSubstitution([FindPackageShare('model_client'), 'config', 'act.yaml'])), DeclareLaunchArgument('model_url', default_value='ws://127.0.0.1:8000'), DeclareLaunchArgument('instruction', default_value='Move the cube to the target'), Node(package='model_client', executable='client', output='screen', arguments=['--config', LaunchConfiguration('config'), '--url', LaunchConfiguration('model_url'), '--instruction', LaunchConfiguration('instruction')])])
