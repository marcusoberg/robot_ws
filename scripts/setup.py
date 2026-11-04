"""Install Python packages and build the ROS interface/node packages."""

import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['external/robot_kinematics', 'src/robot_description', 'src/robot_data',
            'src/robot_server', 'src/robot_teleop', 'src/robot_path', 'src/model_client']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    if not args.build_only:
        command = [sys.executable, '-m', 'pip', 'install', '--no-build-isolation']
        for package in PACKAGES:
            command.extend(['-e', package])
        subprocess.run(command, cwd=ROOT, check=True)
    if args.ros:
        subprocess.run([
            'colcon', 'build', '--base-paths', 'src', '--build-base', 'build/ros-v2',
            '--install-base', 'install/ros-v2', '--merge-install', '--symlink-install',
            '--executor', 'sequential', '--cmake-args', f'-DPython3_EXECUTABLE={sys.executable}',
        ], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
