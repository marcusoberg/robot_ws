"""Move PAROL6 to six joint angles through an already-running robot server."""

import argparse
import math
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from robot_description.spec import parol6
from robot_path.joint_move import JointMove
from robot_runtime_native._native import Parol6Serial
from robot_server_node.client import Source


class MoveJoint(Source):
    def __init__(self, arm_target, duration=4., gripper='pneumatic'):
        self.profile = parol6(gripper, scene='robot_only')
        # Validate the requested endpoint and duration before acquiring control.
        target = JointMove(self.profile.spec, self.profile.initial, arm_target, duration)
        super().__init__('move_joint', self.profile.spec)
        self.motion_executor = SingleThreadedExecutor(context=self.context)
        self.motion_executor.add_node(self)
        self.arm_target = target.target[:6]
        self.duration = target.duration
        self.status_received_at = None
        self.status_count = 0
        self.status_timeout = self.profile.source_timeout_ns / 1e9
        self.plan = None
        self.completed = False
        self.completion_samples = 0
        self.last_completion_status = 0

    def receive_status(self, status):
        if status.action_spec_id != self.spec.identity:
            raise ValueError('Robot server contract differs from this PAROL6 pose command')
        self.status_received_at = time.monotonic()
        self.status_count += 1
        super().receive_status(status)

    def wait_for(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while not predicate():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError(message)
            self.motion_executor.spin_once(timeout_sec=.01)

    def require_feedback(self):
        if (self.status_received_at is None
                or time.monotonic() - self.status_received_at > self.status_timeout):
            raise TimeoutError('Robot server feedback is stale')

    def acknowledged_control(self, operation):
        self.control(operation)
        command_id = self.pending_commands[-1][0]
        self.wait_for(lambda: self.status.command_id == command_id, 5.,
                      f'No server acknowledgement for {operation}')
        if not self.status.command_accepted:
            raise RuntimeError(self.status.command_result)

    def run(self):
        self.wait_for(lambda: (self.status is not None
                              and self.commands.get_subscription_count()
                              and self.goals.get_subscription_count()),
                      10., 'No compatible robot server discovered')
        self.require_feedback()
        if self.status.mode != 'Ready' or self.status.source or self.status.session:
            raise RuntimeError(
                f'Server must be Ready and unowned; current state is {self.status.mode}: '
                f'{self.status.reason}. Pause an active source with "pixi run pause" first.'
            )
        self.server_identity = (self.status.instance, self.status.episode)
        self.acknowledged_control('activate')
        initial = list(self.status.measured_positions)
        initial[-1] = self.status.commanded_reference[-1]
        self.plan = JointMove(self.spec, initial, self.arm_target, self.duration)
        self.started_at = time.monotonic()
        if self.native_motion:
            self.begin_move(self.plan.target, self.duration)
        self.get_logger().info(
            f'Moving to {np.rad2deg(self.arm_target).round(6).tolist()} degrees '
            f'in {self.duration:g} s'
        )
        # Serial owns trajectory time; this timer only renews its producer lease.
        period = self.spec.period_ns if self.native_motion else self.profile.execution_period_ns
        timer = self.create_timer(period / 1e9, self.publish_goal)
        try:
            self.wait_for(lambda: self.completed,
                          self.duration + self.profile.stop_timeout_ns / 1e9 + 1.,
                          'Timed out waiting for the joint move to finish')
        finally:
            self.destroy_timer(timer)
        self.hold()
        self.get_logger().info('Target reached; robot is Ready and holding')

    def publish_goal(self):
        self.require_feedback()
        if (self.status.instance, self.status.episode) != self.server_identity:
            raise RuntimeError('Robot server restarted during the joint move')
        if not self.available():
            raise RuntimeError(f'Move interrupted: {self.status.mode}: {self.status.reason}')
        elapsed = time.monotonic() - self.started_at
        if self.native_motion:
            self.motion_heartbeat()
            finished = self.motion_complete
        else:
            self.send(self.plan.at(elapsed), velocities=self.plan.velocity_at(elapsed))
            finished = elapsed >= self.duration
        if finished:
            # Count distinct feedback samples, not repeated reads of one status.
            if self.status_count != self.last_completion_status:
                self.last_completion_status = self.status_count
                error = np.max(np.abs(
                    np.asarray(self.status.measured_positions[:6]) - self.arm_target
                ))
                self.completion_samples = self.completion_samples + 1 if error <= .001 else 0
            if self.completion_samples >= 3:
                self.completed = True
            elif elapsed > self.duration + self.profile.stop_timeout_ns / 1e9:
                raise TimeoutError('Robot did not reach the requested joint pose')

    def hold(self):
        # A fault can release ownership while the last received status still
        # names our session. Refresh it before issuing a session-bound pause.
        if self.session:
            previous_count = self.status_count
            self.wait_for(lambda: self.status_count > previous_count, 1.,
                          'No fresh server status available to confirm hold')
        if self.session:
            self.acknowledged_control('pause')
            self.wait_for(lambda: self.status.mode == 'Ready' and not self.status.session,
                          self.profile.stop_timeout_ns / 1e9,
                          'Server did not confirm Ready after the hold request')
        elif self.status is not None and self.status.mode == 'Fault':
            self.get_logger().error(f'Server fault: {self.status.reason}')

    def destroy_node(self):
        self.motion_executor.remove_node(self)
        self.motion_executor.shutdown()
        super().destroy_node()


def positive_seconds(value):
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0:
        raise argparse.ArgumentTypeError('time must be finite and positive')
    return duration


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--standby', action='store_true', help='return to the post-homing pose')
    target.add_argument('--degrees', nargs=6, type=float, metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'))
    parser.add_argument('--time', type=positive_seconds, default=4., help='move duration in seconds (default: 4)')
    parser.add_argument('--gripper', choices=('pneumatic', 'msg_ai_100'), default='pneumatic')
    return parser.parse_args(arguments)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main(arguments=None):
    args = parse_arguments(arguments)
    if args.standby:
        target = Parol6Serial.standby()
    else:
        target = np.deg2rad(args.degrees)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    result = 0
    try:
        node = MoveJoint(target, args.time, args.gripper)
        node.run()
    except KeyboardInterrupt:
        print('Joint move interrupted; requesting hold', file=sys.stderr)
        result = 130
    except Exception as error:
        print(str(error), file=sys.stderr)
        result = 1
    finally:
        if node is not None:
            try:
                node.hold()
            except Exception as error:
                print(f'Could not confirm hold: {error}', file=sys.stderr)
                result = 1
            finally:
                node.destroy_node()
        rclpy.try_shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
