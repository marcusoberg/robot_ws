from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='example'),
        Node(
            package='robot_examples',
            executable='send_joint',
            output='screen',
            arguments=['--source', LaunchConfiguration('source')])
    ])
