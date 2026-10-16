"""SO101 leader and explicitly stepped PAROL6 simulation, with optional collection."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    teleop = Path(get_package_share_directory("robot_teleop"))
    server = Path(get_package_share_directory("robot_server"))
    workspace = os.environ.get("PIXI_PROJECT_ROOT")
    if not workspace:
        raise RuntimeError("Launch from the workspace Pixi shell so the LeRobot environment and output paths are explicit")
    root = Path(workspace)
    args = [
        DeclareLaunchArgument("port", description="SO101 leader serial port; no automatic device selection"),
        DeclareLaunchArgument("leader_id", default_value="so101-leader"),
        DeclareLaunchArgument("calibration_dir", default_value=""),
        DeclareLaunchArgument("config", default_value=str(teleop/"config/so101_parol6.yaml")),
        DeclareLaunchArgument("leader_urdf", default_value=str(server/"assets/so101/so101_new_calib.urdf")),
        DeclareLaunchArgument("model_python", default_value=str(root/"environments/models/.venv/bin/python")),
        DeclareLaunchArgument("output", default_value=str(root/"training/datasets")),
        DeclareLaunchArgument("start_server", default_value="true"),
        DeclareLaunchArgument("deployment", default_value=str(server/"config/parol6.yaml")),
        DeclareLaunchArgument("foxglove", default_value="true"),
    ]
    return LaunchDescription(args+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(server/"launch/server.launch.py")),
            condition=IfCondition(LaunchConfiguration("start_server")),
            launch_arguments={"deployment":LaunchConfiguration("deployment"),"backend":"mujoco",
                              "mode":"stepped","foxglove":LaunchConfiguration("foxglove")}.items()),
        Node(package="robot_teleop", executable="so101", output="screen",
            arguments=["--port",LaunchConfiguration("port"),"--leader-id",LaunchConfiguration("leader_id"),
                       "--calibration-dir",LaunchConfiguration("calibration_dir"),
                       "--config",LaunchConfiguration("config"),"--urdf",LaunchConfiguration("leader_urdf"),
                       "--python",LaunchConfiguration("model_python"),"--output",LaunchConfiguration("output")])
    ])
