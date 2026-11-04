"""Select the Python API serial node or the inactive simulation placeholder."""

import shlex
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_server_node.ownership import stop_existing_server


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='simulation', choices=['simulation', 'serial']),
        DeclareLaunchArgument('serial_port', default_value='/dev/cu.usbmodem3074388334331'),
        DeclareLaunchArgument('api_port', default_value='5001'),
        DeclareLaunchArgument('gripper', default_value='pneumatic', choices=['pneumatic', 'msg_ai_100']),
        DeclareLaunchArgument('restart', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_server),
    ])


def start_server(context):
    if LaunchConfiguration('restart').perform(context) == 'true':
        stop_existing_server()
    backend = LaunchConfiguration('backend').perform(context)
    parameters = []
    if backend == 'serial':
        parameters = [{
            'serial_port': LaunchConfiguration('serial_port').perform(context),
            'api_port': int(LaunchConfiguration('api_port').perform(context)),
            'gripper': LaunchConfiguration('gripper').perform(context),
        }]
    return [Node(package='robot_server', executable=f'{"serial" if backend == "serial" else "sim"}_node.py',
                 prefix=[shlex.quote(sys.executable)], parameters=parameters, output='screen')]
