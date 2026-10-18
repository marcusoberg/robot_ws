import re
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import robot_description
from robot_description.spec import load_profile


def nodes(context):
    profile = load_profile(LaunchConfiguration('robot').perform(context),
                           gripper=LaunchConfiguration('gripper').perform(context))
    assets = Path(robot_description.__file__).parent / 'assets'
    assets_uri = 'package://robot_description/' + profile.description.parent.relative_to(assets).as_posix() + '/'
    description = ET.parse(profile.description).getroot()
    for mesh in description.findall('.//mesh'):
        filename = mesh.attrib['filename']
        if '://' not in filename and not Path(filename).is_absolute():
            mesh.set('filename', assets_uri + filename)
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': ET.tostring(description, encoding='unicode'), 'use_sim_time': True}],
                     remappings=[('robot_description','/robot/urdf')])
    bridge = Node(package='foxglove_bridge', executable='foxglove_bridge',
                  condition=IfCondition(LaunchConfiguration('foxglove')),
                  parameters=[{'address': '127.0.0.1', 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                               'asset_uri_allowlist': ['^' + re.escape(assets_uri) + r'.+\.(?:stl|STL)$']}])
    return [
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Foxglove bridge exited'))])),
        publisher, bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='so101'),
        DeclareLaunchArgument('gripper', default_value='pneumatic'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('port', default_value='8766'),
        OpaqueFunction(function=nodes),
    ])
