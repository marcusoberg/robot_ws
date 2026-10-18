import argparse
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from robot_description.spec import so101
from robot_server_node.client import Source
from .leader import SerialLeader
from .keyboard import Keyboard


class LeaderSource(Source):
    def __init__(self, port, calibration, keyboard=True, ik_config=None):
        self.ik = None
        if ik_config:
            from .ik import FKIKMapping
            self.ik = FKIKMapping(ik_config)
        super().__init__('so101_source', self.ik.spec if self.ik else so101().spec)
        if self.ik:
            self.leader = SerialLeader(port, calibration, self.ik.leader.spec, map_positions=False)
        else:
            self.leader = SerialLeader(port, calibration, self.spec)
        self.keys = Keyboard(self, self.command, keyboard)
        self.preview = self.create_publisher(JointState, '/teleop/target', 1)
        self.create_subscription(Bool, '/teleop/engage', self.engage, 1)
        self.create_timer(self.spec.period_ns / 1e9, self.sample)
        self.get_logger().info('Paused. SPACE activates/pauses; r records, s saves, d discards, x resets.')
        if self.ik:
            self.get_logger().info(f'FK → IK enabled: {self.ik.follower.description}; joints: {", ".join(self.spec.joints)}')

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
        if self.ik:
            raw, _ = self.leader.read_raw()
            seed = self.status.measured_positions if self.status is not None else self.ik.reference
            goal = self.ik.map(raw, self.leader.calibration, seed)
        else:
            goal, _ = self.leader.read()
        self.preview.publish(JointState(name=list(self.spec.joints), position=goal.tolist()))
        self.send(goal)

    def destroy_node(self):
        self.keys.close()
        self.leader.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--keyboard', choices=('auto', 'off'), default='auto')
    parser.add_argument('--ik-config', default='', help='Optional FK/IK TOML configuration; omitted uses joint mapping')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = LeaderSource(args.port, args.calibration, args.keyboard != 'off', args.ik_config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
