import copy
from pathlib import Path
import shlex
import threading
import tomllib

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_server.msg import Control, JointGoal, Status
from robot_description.spec import parol6
from robot_path.move_joint import MoveJoint, parse_arguments


POSE = np.deg2rad([90., -144.683, 108.171, 5., 0., 90.])
STANDBY = np.deg2rad([90., -90., 180., 0., 0., 180.])


class MotionServer(Node):
    """An ideal follower in an isolated ROS domain; no serial or robot access."""

    def __init__(self):
        super().__init__('pose_test_server', start_parameter_services=False)
        profile = parol6(scene='robot_only')
        positions = [*STANDBY, profile.initial[-1]]
        self.state = Status(mode='Ready', instance='pose-test', episode=1,
                            action_spec_id=profile.spec.identity,
                            joints=list(profile.spec.joints),
                            measured_positions=positions, commanded_reference=positions)
        self.controls = []
        self.goals = []
        self.interrupt_mode = None
        self.publisher = self.create_publisher(Status, '/robot/status', 1)
        self.create_subscription(Control, '/robot/control', self.control, 10)
        self.create_subscription(JointGoal, '/robot/joint_goal', self.goal, 1)
        self.create_timer(.01, self.publish)

    def publish(self):
        self.publisher.publish(self.state)

    def control(self, message):
        self.controls.append(message.operation)
        self.state.command_id = message.command_id
        self.state.command_accepted = True
        if message.operation == 'activate':
            self.state.mode = 'Running'
            self.state.source = message.source
            self.state.session = 123
        elif message.operation == 'pause':
            self.state.mode = 'Ready'
            self.state.source = ''
            self.state.session = 0
        self.publish()

    def goal(self, message):
        self.goals.append(copy.deepcopy(message))
        self.state.measured_positions = message.positions
        self.state.commanded_reference = message.positions
        self.state.sequence = message.sequence
        if self.interrupt_mode and len(self.goals) == 3:
            self.state.mode = self.interrupt_mode
            self.state.source = ''
            self.state.session = 0
            self.state.reason = 'operator interrupted the move'
        self.publish()


@pytest.fixture
def server(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '74')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    rclpy.init(args=[])
    node = MotionServer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()
        assert not thread.is_alive()


def test_pose_then_standby_release_ownership_and_hold(server):
    for target in (POSE, STANDBY):
        node = MoveJoint(target, duration=.15)
        try:
            node.run()
            assert node.completed
            np.testing.assert_array_equal(server.state.measured_positions[:6], target)
            assert server.state.measured_positions[-1] == .0035
            assert server.state.mode == 'Ready' and not server.state.session
        finally:
            node.hold()
            node.destroy_node()
    assert server.controls == ['activate', 'pause', 'activate', 'pause']
    assert len(server.goals) >= 30
    assert all(len(goal.velocities) == 7 and goal.velocities[-1] == 0 for goal in server.goals)
    assert any(any(goal.velocities) for goal in server.goals)
    assert not any(server.goals[-1].velocities)


@pytest.mark.parametrize('mode', ['Idle', 'Fault', 'Running'])
def test_move_does_not_activate_an_unready_or_owned_robot(server, mode):
    server.state.mode = mode
    if mode == 'Running':
        server.state.source = 'existing_path'
        server.state.session = 456
    node = MoveJoint(POSE)
    try:
        with pytest.raises(RuntimeError, match='Ready and unowned'):
            node.run()
        assert not server.controls
        assert not server.goals
    finally:
        node.destroy_node()


@pytest.mark.parametrize('mode', ['Ready', 'Fault'])
def test_interrupted_move_exits_without_reactivation(server, mode):
    server.interrupt_mode = mode
    node = MoveJoint(POSE, duration=.2)
    try:
        with pytest.raises(RuntimeError, match='Move interrupted'):
            node.run()
        assert server.controls == ['activate']
        assert not node.completed
    finally:
        node.hold()
        node.destroy_node()


def test_pixi_pose_tasks_preserve_the_requested_angles_and_time():
    root = Path(__file__).resolve().parents[1]
    tasks = tomllib.loads((root / 'pixi.toml').read_text())['feature']['ros']['tasks']
    pose = parse_arguments(shlex.split(tasks['pose'])[3:])
    assert pose.degrees == [90., -144.683, 108.171, 5., 0., 90.]
    assert pose.time == 4.
    standby = parse_arguments(shlex.split(tasks['standby'])[3:])
    assert standby.standby and standby.time == 4.
