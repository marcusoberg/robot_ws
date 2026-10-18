import argparse
from pathlib import Path
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from robot_description.spec import so101
from robot_server_node.client import Source
from .leader import SerialLeader
from .keyboard import Keyboard


class LeaderSource(Source):
    def __init__(self, port, calibration, keyboard=True):
        profile = so101()
        super().__init__('so101_source', profile.spec)
        self.leader = SerialLeader(port, calibration, self.spec)
        self.keys = Keyboard(self, self.command, keyboard)
        self.preview = self.create_publisher(JointState, '/teleop/target', 1)
        self.create_subscription(Bool, '/teleop/engage', self.engage, 1)
        self.create_timer(self.spec.period_ns / 1e9, self.sample)
        self.failure = None
        self.get_logger().info('Paused. SPACE activates/pauses; r records, s saves, d discards, x resets.')

    def command(self, operation):
        if operation == 'toggle':
            self.toggle()
        else:
            self.control(operation)

    def engage(self, message):
        if message.data and not self.session:
            self.control('activate')
        elif not message.data and self.session:
            self.control('pause')

    def session_changed(self):
        if self.status:
            self.get_logger().info(f'{self.status.mode}: {self.status.reason or self.status.source}')

    def sample(self):
        try:
            goal, acquired = self.leader.read()
            if time.monotonic_ns() - acquired >= 300_000_000:
                raise ValueError('Leader sample expired during acquisition')
            self.preview.publish(JointState(name=list(self.spec.joints), position=goal.tolist()))
            if self.available():
                self.send(goal)
            self.failure = None
        except Exception as exc:
            if self.session:
                self.control('pause')
            if self.failure != str(exc):
                self.get_logger().error(f'Leader source unavailable: {exc}')
                self.failure = str(exc)

    def destroy_node(self):
        self.keys.close()
        self.leader.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--keyboard', choices=('auto', 'off'), default='auto')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = LeaderSource(args.port, args.calibration, args.keyboard != 'off')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
