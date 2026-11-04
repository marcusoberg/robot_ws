"""Exercise the native serial boundary through a PTY, without a physical robot."""

import math
import os
import select
import threading
import time

import pytest

pty = pytest.importorskip('pty')
native = pytest.importorskip('robot_runtime_native._native')
if not hasattr(native, 'Parol6Serial'):
    pytest.skip('PAROL6 serial runtime is not built', allow_module_level=True)

HOME_COUNTS = [10240, -32000, 57905, 0, 0, 32000]
COUNTS_PER_RAD = [200 * 32 * ratio / (2 * math.pi)
                  for ratio in (6.4, 20., 20. * 38. / 42., 4., 4., 10.)]


def packet_positions(packet):
    return [int.from_bytes(packet[4 + 3*i:7 + 3*i], 'big', signed=True)
            for i in range(6)]


class Firmware:
    """Stock reply-before-dispatch ordering and command-156 position response."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.pressed = True
        self.disabled = True
        self.steps = [0] * 6
        self.homed = 3
        self.home_at = None
        self.packets = []
        self.error = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        pending = b''
        try:
            while not self.done.is_set():
                if not select.select([self.master], [], [], .01)[0]:
                    continue
                pending += os.read(self.master, 4096)
                while len(pending) >= 56:
                    packet, pending = pending[:56], pending[56:]
                    self.packets.append(packet)
                    command = packet[40]
                    if self.home_at is not None and time.monotonic() >= self.home_at:
                        self.steps = HOME_COUNTS.copy()
                        self.homed = 255
                        self.home_at = None
                    reply = bytearray([255, 255, 255, 56])
                    for step in self.steps:
                        reply.extend(step.to_bytes(3, 'big', signed=True))
                    reply.extend(bytes(18))
                    reply.extend([self.homed, 7 if self.pressed else 15,
                                  3, 3, 0, 10, 100, command])
                    reply.extend(bytes(9))
                    reply.extend([212, 1, 2])
                    os.write(self.master, reply)
                    if command == 102:
                        self.disabled = True
                        self.home_at = None
                    elif command == 101:
                        self.disabled = False
                    elif command == 100 and not self.disabled:
                        self.homed = 3
                        self.steps = [0] * 6
                        self.home_at = time.monotonic() + .05
                    elif command == 156 and not self.disabled:
                        # Firmware speed = (error / .01 + feed-forward) / 2.
                        # Integrate one .01-second exchange here.
                        for i, target in enumerate(packet_positions(packet)):
                            velocity = int.from_bytes(packet[22 + 3*i:25 + 3*i], 'big', signed=True)
                            self.steps[i] += int((target - self.steps[i]) / 2 + velocity * .005)
        except Exception as error:
            self.error = error

    def close(self):
        self.done.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)
        assert not self.thread.is_alive()
        assert self.error is None


def wait(robot, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        robot.keepalive()
        state = robot.observe()
        if predicate(state):
            return state
        time.sleep(.005)
    pytest.fail(f'Timed out: {state.mode}: {state.reason}')


@pytest.fixture
def connected():
    firmware = Firmware()
    robot = None
    try:
        robot = native.Parol6Serial(firmware.port, 115200)
        wait(robot, lambda state: state.mode == 'Idle')
        firmware.pressed = False
        wait(robot, lambda state: state.estop_verified)
        assert robot.home(), robot.observe().reason
        wait(robot, lambda state: state.mode == 'Ready')
        yield robot, firmware
    finally:
        if robot is not None:
            robot.close()
        firmware.close()


@pytest.mark.parametrize('direction', [1, -1])
def test_position_targets_reach_wire_without_rate_clipping(connected, direction):
    robot, firmware = connected
    goal = [q + direction * .02 for q in robot.observe().position]
    expected = [round(q * ratio) for q, ratio in zip(goal, COUNTS_PER_RAD)]
    assert robot.start()
    first = len(firmware.packets)
    assert robot.submit(goal), robot.observe().reason
    wait(robot, lambda state: any(p[40] == 156 for p in firmware.packets[first:]))
    packets = [p for p in firmware.packets[first:] if p[40] == 156]
    assert any(packet_positions(packet) == expected for packet in packets)
    wait(robot, lambda state: max(abs(a - b) for a, b in zip(state.position, goal)) < .0005)
    assert robot.observe().mode == 'Running', robot.observe().reason


def test_planned_trajectory_above_point_two_rad_per_second(connected):
    robot, firmware = connected
    initial = list(robot.observe().position)
    assert robot.start()
    started = time.monotonic()
    for sample in range(16):
        deadline = started + sample * .05
        time.sleep(max(0, deadline - time.monotonic()))
        goal = initial.copy()
        goal[0] += sample * .05 * .314
        assert robot.submit(goal), robot.observe().reason
    wait(robot, lambda state: abs(state.position[0] - goal[0]) < .0005)
    assert robot.observe().mode == 'Running', robot.observe().reason
    robot.pause()
    wait(robot, lambda state: state.mode == 'Ready')
    assert not firmware.disabled


@pytest.mark.parametrize('invalid', ['nan', 'joint_limit', 'tracking_error'])
def test_rejected_targets_still_disable(connected, invalid):
    robot, firmware = connected
    assert robot.start()
    goal = list(robot.observe().position)
    if invalid == 'nan':
        goal[0] = float('nan')
    elif invalid == 'joint_limit':
        goal[0] = 10.
    else:
        goal[0] += .04
    assert not robot.submit(goal)
    wait(robot, lambda state: state.mode == 'Fault' and state.command_echo == 102)
    assert firmware.disabled


def test_estop_still_disables_during_tracking(connected):
    robot, firmware = connected
    assert robot.start()
    assert robot.submit(list(robot.observe().position))
    wait(robot, lambda state: state.command_echo == 156)
    firmware.pressed = True
    wait(robot, lambda state: state.mode == 'Fault' and state.command_echo == 102)
    assert 'E-stop' in robot.observe().reason
    assert firmware.disabled


def test_four_second_pose_and_return_to_standby(connected):
    from robot_description.spec import parol6
    from robot_path.joint_move import JointMove

    robot, firmware = connected
    profile = parol6(scene='robot_only')
    pose = [math.radians(q) for q in (90., -144.683, 108.171, 5., 0., 90.)]
    for target in (pose, native.Parol6Serial.standby()):
        initial = [*robot.observe().position, profile.initial[-1]]
        move = JointMove(profile.spec, initial, target, 4.)
        assert robot.start()
        started = time.monotonic()
        while True:
            elapsed = time.monotonic() - started
            assert robot.submit(move.at(elapsed)[:6].tolist(),
                                move.velocity_at(elapsed)[:6].tolist()), robot.observe().reason
            if elapsed >= move.duration:
                break
            time.sleep(profile.execution_period_ns / 1e9)
        wait(robot, lambda state: max(abs(a - b) for a, b in zip(state.position, target)) < .001)
        robot.pause()
        wait(robot, lambda state: state.mode == 'Ready')
        assert not firmware.disabled
    assert firmware.steps == pytest.approx(HOME_COUNTS, abs=2)


def test_feedforward_velocity_packet_and_position_only_fallback(connected):
    robot, firmware = connected
    velocities = [.3, -.3, .4, -.5, .6, -.7]
    expected = [round(v * ratio) for v, ratio in zip(velocities, COUNTS_PER_RAD)]
    assert robot.start()
    first = len(firmware.packets)
    assert robot.submit(list(robot.observe().position), velocities)
    def matching_packet(state):
        return any(p[40] == 156 and
                   [int.from_bytes(p[22 + 3*i:25 + 3*i], 'big', signed=True)
                    for i in range(6)] == expected
                   for p in firmware.packets[first:])
    wait(robot, matching_packet)
    first = len(firmware.packets)
    assert robot.submit(list(robot.observe().position))
    wait(robot, lambda state: any(p[40] == 156 and p[22:40] == bytes(18)
                                  for p in firmware.packets[first:]))


@pytest.mark.parametrize('velocities', [[0.] * 5, [float('nan')] * 6, [1e10] * 6])
def test_invalid_feedforward_is_rejected_at_native_boundary(connected, velocities):
    robot, firmware = connected
    assert robot.start()
    assert not robot.submit(list(robot.observe().position), velocities)
    wait(robot, lambda state: state.mode == 'Fault' and state.command_echo == 102)
    assert firmware.disabled
