"""Run the execution server and robot model publisher."""

import shlex
import sys
import time
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RosNode
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from robot_server_node.ownership import existing_server, stop_existing_server


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("robot", default_value="so101"),
        DeclareLaunchArgument("gripper", default_value="pneumatic"),
        DeclareLaunchArgument("scene", default_value="pick_place"),
        DeclareLaunchArgument(
            "backend", default_value="simulation", choices=["simulation", "serial"]
        ),
        DeclareLaunchArgument("cameras", default_value="false"),
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/cu.usbmodem3074388334331"
        ),
        DeclareLaunchArgument("tool_isolated", default_value="false"),
        DeclareLaunchArgument("paced", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("foxglove_port", default_value="8766"),
        DeclareLaunchArgument("recordings", default_value="recordings"),
        DeclareLaunchArgument("restart", default_value="false", choices=["true", "false"]),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=start_or_attach)])


def start_or_attach(context):
    if LaunchConfiguration("restart").perform(context) == "true":
        stop_existing_server()
    owner = existing_server()
    if owner is not None:
        for name in ("robot", "gripper", "backend", "scene", "serial_port"):
            requested = LaunchConfiguration(name).perform(context)
            if (
                name == "serial_port"
                and LaunchConfiguration("backend").perform(context) != "serial"
            ):
                continue
            if name in owner and owner[name] != requested:
                raise RuntimeError(
                    f"Existing server {name}={owner[name]!r}; requested {requested!r}"
                )
        terminal = Node(
            package="robot_server",
            executable="attach.py",
            prefix=[shlex.quote(sys.executable)],
            output="screen",
            parameters=[
                {
                    "robot": LaunchConfiguration("robot"),
                    "gripper": LaunchConfiguration("gripper"),
                    "scene": LaunchConfiguration("scene"),
                    "backend": LaunchConfiguration("backend"),
                    "instance": owner.get("instance", ""),
                }
            ],
        )
        publish_tf, publish_model = missing_visual_publishers()
        return [
            LogInfo(msg="Robot server already running; attaching terminal"),
            shutdown_when_exited(terminal),
            terminal,
            visualization(publish_tf, publish_model),
        ]

    # Resolve executables before starting the serial owner. A missing visual
    # executable must not leave a half-launched server owning the port.
    for package, executable in (
        ("robot_server", "server"),
        ("robot_server", "model.py"),
        ("robot_state_publisher", "robot_state_publisher"),
    ):
        ExecutableInPackage(package=package, executable=executable).perform(context)
    server = Node(
        package="robot_server",
        executable="server",
        prefix=[shlex.quote(sys.executable)],
        output="screen",
        parameters=[
            {
                "robot": LaunchConfiguration("robot"),
                "gripper": LaunchConfiguration("gripper"),
                "scene": LaunchConfiguration("scene"),
                "backend": LaunchConfiguration("backend"),
                "cameras": ParameterValue(
                    LaunchConfiguration("cameras"), value_type=bool
                ),
                "serial_port": ParameterValue(
                    LaunchConfiguration("serial_port"), value_type=str
                ),
                "tool_isolated": ParameterValue(
                    LaunchConfiguration("tool_isolated"), value_type=bool
                ),
                "paced": ParameterValue(LaunchConfiguration("paced"), value_type=bool),
                "recordings": LaunchConfiguration("recordings"),
            }
        ],
    )
    return [shutdown_when_exited(server), server, visualization()]


def visualization(publish_tf="true", publish_model="true"):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_server"),
                    "launch",
                    "visualization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "publish_tf": publish_tf,
            "publish_model": publish_model,
            "robot": LaunchConfiguration("robot"),
            "gripper": LaunchConfiguration("gripper"),
            "scene": LaunchConfiguration("scene"),
            "backend": LaunchConfiguration("backend"),
            "foxglove": LaunchConfiguration("foxglove"),
            "port": LaunchConfiguration("foxglove_port"),
        }.items(),
    )


def missing_visual_publishers():
    """Brief read-only discovery prevents duplicate model and TF publishers."""
    context = Context()
    rclpy.init(args=[], context=context)
    node = RosNode(
        "robot_launch_discovery", context=context, start_parameter_services=False
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        return (
            "false" if node.count_publishers("/robot/urdf") else "true",
            "false" if node.count_publishers("/robot/model") else "true",
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def shutdown_when_exited(process):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="Robot server session ended"))],
        )
    )
