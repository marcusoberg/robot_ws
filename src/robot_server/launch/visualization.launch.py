import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_description.spec import so101


def generate_launch_description():
    profile = so101()
    assets_uri = 'package://robot_description/so101/assets/'
    description = profile.description.read_text().replace('filename="assets/', 'filename="' + assets_uri)
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': description, 'use_sim_time': True}],
                     remappings=[('robot_description','/robot/urdf')])
    bridge = Node(package='foxglove_bridge', executable='foxglove_bridge',
                  condition=IfCondition(LaunchConfiguration('foxglove')),
                  parameters=[{'address': '127.0.0.1', 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                               'asset_uri_allowlist': ['^' + re.escape(assets_uri) + r'[^/]+\.stl$']}])
    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('port', default_value='8766'),
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Foxglove bridge exited'))])),
        publisher, bridge,
    ])
