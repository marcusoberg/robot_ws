import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='Launch the SO101 leader; start the server and Foxglove separately')
    parser.add_argument('--port')
    parser.add_argument('--calibration')
    parser.add_argument('--recordings')
    parser.add_argument('--foxglove-port')
    parser.add_argument('--keyboard', choices=('auto', 'off'))
    args, launch_args = parser.parse_known_args()

    if any(':=' not in item for item in launch_args):
        parser.error('Use --port /dev/... or ROS launch arguments such as port:=/dev/...')

    for option, name in [('port', 'port'), ('calibration', 'calibration'), ('recordings', 'output'), ('foxglove_port', 'foxglove_port'),
                         ('keyboard', 'keyboard')]:
        value = getattr(args, option)
        if value is not None:
            if any(item.startswith(name + ':=') for item in launch_args): parser.error(f'{name} supplied twice')
            launch_args.append(f'{name}:={value}')
    if not any(item.startswith('port:=') and item != 'port:=' for item in launch_args):
        parser.error('An explicit serial port is required: --port /dev/... or port:=/dev/...')
    command = [sys.executable, '-c', 'from ros2cli.cli import main; main()', 'launch', 'robot_teleop', 'so101_so101.launch.py', *launch_args]
    print('Launching SO101 teleoperation; SPACE activates/pauses the leader.', flush=True)
    try:
        return subprocess.call(command)
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
