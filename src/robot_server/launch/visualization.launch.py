import re
import shlex
import sys
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
                           gripper=LaunchConfiguration('gripper').perform(context),
                           scene=LaunchConfiguration('scene').perform(context))
    use_sim_clock = (LaunchConfiguration('backend').perform(context) == 'simulation'
                     and profile.task != 'robot_only')
    assets = Path(robot_description.__file__).parent / 'assets'
    assets_uri = 'package://robot_description/' + profile.description.parent.relative_to(assets).as_posix() + '/'
    description = ET.parse(profile.description).getroot()
    for mesh in description.findall('.//mesh'):
        filename = mesh.attrib['filename']
        if '://' not in filename and not Path(filename).is_absolute():
            mesh.set('filename', assets_uri + filename)
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     condition=IfCondition(LaunchConfiguration('publish_tf')),
                     parameters=[{'robot_description': ET.tostring(description, encoding='unicode'), 'use_sim_time': use_sim_clock}],
                     remappings=[('robot_description','/robot/urdf')])
    model = Node(package='robot_server', executable='model.py',
                 condition=IfCondition(LaunchConfiguration('publish_model')),
                 prefix=[shlex.quote(sys.executable)],
                 parameters=[{'description_file': str(profile.description)}])
    bridge = Node(package='foxglove_bridge', executable='foxglove_bridge',
                  condition=IfCondition(LaunchConfiguration('foxglove')),
                  parameters=[{'address': '127.0.0.1', 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                               'asset_uri_allowlist': ['^' + re.escape(assets_uri) + r'.+\.(?:stl|STL)$']}])
    return [
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Foxglove bridge exited'))])),
        publisher, model, bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='parol6'),
        DeclareLaunchArgument('gripper', default_value='pneumatic'),
        DeclareLaunchArgument('scene', default_value='robot_only'),
        DeclareLaunchArgument('backend', default_value='simulation'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_model', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('port', default_value='8766'),
        OpaqueFunction(function=nodes),
    ])
