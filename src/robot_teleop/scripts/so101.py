#!/usr/bin/env python3
"""ROS orchestration: acquire leader samples, solve IK, publish follower targets."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from robot_description.spec import so101
from robot_kinematics import JointStream
from robot_server_core.motion_trace import MotionTrace
from robot_server_node.client import Source
from robot_teleop.fk_ik_mapping import FKIKMapping
from robot_teleop.leader import LeaderFeedbackTimeout, SerialLeader


class LeaderSource(Source):
    def __init__(self, port, calibration, ik_config):
        self.ik = FKIKMapping(ik_config)
        super().__init__('so101_source', self.ik.spec)
        profile = so101()
        self.feedback_timeout = profile.source_timeout_ns / 1e9
        self.startup_timeout = profile.stop_timeout_ns / 1e9
        self.leader = SerialLeader(port, calibration, self.ik.leader_spec)
        self.leader.start_reader()
        self.last_processed_sample = 0
        self.pausing = False
        self.initialized = self.ik.startup_duration == 0
        self.startup_identity = None
        self.startup_initial = None
        self.startup_target = self.ik.reference.copy()
        self.last_ik_goal = self.ik.reference.copy()
        self.last_ik_feedback = 0.
        self.joint_stream = None
        self.ik_worker = ThreadPoolExecutor(max_workers=1)
        self.ik_future = None
        self.ik_request = None
        self.motion_trace = MotionTrace('teleop', self.spec.joints)
        self.trace_session = 0
        self.preview = self.create_publisher(JointState, '/teleop/target', 1)
        self.create_subscription(Bool, '/teleop/engage', self.engage, 1)
        self.create_timer(self.spec.period_ns / 1e9, self.sample)
        self.create_timer(profile.execution_period_ns / 1e9, self.publish_motion)
        self.get_logger().info('Paused. Use pixi run engage / pixi run disengage.')
        self.get_logger().info(f'FK → IK: {self.ik.follower.description}')
        if self.ik.startup_duration:
            self.get_logger().info(
                f'First engagement moves to {np.rad2deg(self.ik.reference[:6]).round(6).tolist()} '
                f'degrees in {self.ik.startup_duration:g} s, then aligns the current leader frame.'
            )

    def receive_status(self, status):
        super().receive_status(status)
        if self.trace_session:
            self.motion_trace.record('status', mode=status.mode, reason=status.reason,
                                     measured=list(status.measured_positions),
                                     reference=list(status.commanded_reference))
            if status.mode == 'Fault' and self.motion_trace.path is None:
                path = self.motion_trace.save(status.reason)
                self.get_logger().error(f'Teleop command trace saved: {path}')

    def send(self, goal, velocities=(), **motion):
        if not self.available():
            return False
        self.motion_trace.record('send', phase='startup' if self.startup_initial is not None else 'tracking',
                                 positions=list(goal), velocities=list(velocities), **motion)
        return super().send(goal, velocities, **motion)

    def engage(self, message):
        self.control('activate' if message.data else 'pause')

    def pause(self, reason):
        if self.session and not self.pausing:
            self.pausing = True
            self.get_logger().error(f'{reason}; pausing teleop')
            self.control('pause')

    def session_changed(self):
        self.pausing = False
        self.joint_stream = None
        self.startup_initial = None
        self.get_logger().info(f'{self.status.mode}: {self.status.reason or self.status.source}')
        identity = (self.status.instance, self.status.episode)
        if identity != self.startup_identity:
            self.startup_identity = identity
            self.initialized = self.ik.startup_duration == 0
        self.last_ik_goal = np.asarray(self.status.measured_positions).copy()
        passive = np.setdiff1d(np.arange(len(self.spec.joints)), self.ik.follower.active)
        self.last_ik_goal[passive] = np.asarray(self.status.commanded_reference)[passive]
        if not self.session:
            return
        self.trace_session = self.session
        self.motion_trace.reset()
        self.motion_trace.record('activate', session=self.session)
        if not self.initialized:
            self.startup_initial = self.last_ik_goal.copy()
            self.startup_target = self.ik.reference.copy()
            self.startup_target[passive] = self.startup_initial[passive]
            self.startup_started_at = time.monotonic()
            if self.native_motion:
                self.begin_move(self.startup_target, self.ik.startup_duration)
            self.get_logger().info('Moving to the teleop start pose; leader alignment follows at the endpoint.')

    def sample(self):
        if self.status is None or self.pausing:
            return
        try:
            raw, acquired = self.leader.latest_raw()
        except LeaderFeedbackTimeout as error:
            self.pause(str(error))
            return
        if time.monotonic() - acquired / 1e9 >= self.feedback_timeout:
            self.pause('Leader feedback expired')
            return
        if acquired == self.last_processed_sample:
            return
        self.last_processed_sample = acquired
        if not self.initialized:
            self.finish_startup(raw)
            return
        if self.ik_future is None:
            self.ik_request = (self.status.instance, self.session)
            self.ik_sample_time = acquired / 1e9
            self.ik_future = self.ik_worker.submit(
                self.ik.map, raw, self.leader.calibration, self.last_ik_goal.copy())

    def finish_startup(self, raw):
        self.preview.publish(JointState(name=list(self.spec.joints), position=self.startup_target.tolist()))
        if self.startup_initial is None or not self.available():
            return
        elapsed = time.monotonic() - self.startup_started_at
        finished = self.motion_complete if self.native_motion else elapsed >= self.ik.startup_duration
        active = self.ik.follower.active
        error = np.max(np.abs(np.asarray(self.status.measured_positions)[active] - self.startup_target[active]))
        if not finished or error > .001 or self.ik_future is not None:
            return
        # Wait for any previous session's worker before changing its mapping frame.
        self.ik.align_leader(raw, self.leader.calibration)
        self.initialized = True
        self.startup_initial = None
        self.last_ik_goal = self.startup_target.copy()
        self.last_ik_feedback = time.monotonic()
        self.accept_target(self.startup_target)
        self.get_logger().info('Start pose reached. SO101 end-effector frame aligned; teleop running.')

    def accept_target(self, goal):
        if self.native_motion:
            self.send_live(goal, self.spec.period_ns / 1e9)
        else:
            now = time.monotonic()
            if self.joint_stream is None:
                self.joint_stream = JointStream(self.status.measured_positions, now, self.spec.period_ns / 1e9)
            self.joint_stream.update(goal, now)

    def collect_ik(self):
        if self.ik_future is None or not self.ik_future.done():
            return
        future, request = self.ik_future, self.ik_request
        self.ik_future = None
        goal = future.result()
        if self.status is None or request != (self.status.instance, self.session):
            return
        self.motion_trace.record('ik', sample_time=self.ik_sample_time,
                                 positions=goal.tolist(), **self.ik.solver.diagnostics)
        self.last_ik_goal = goal
        self.preview.publish(JointState(name=list(self.spec.joints), position=goal.tolist()))
        self.last_ik_feedback = self.ik_sample_time
        if time.monotonic() - self.ik_sample_time >= self.feedback_timeout:
            self.pause('IK feedback expired')
            return
        if self.available() and not self.pausing:
            self.accept_target(goal)

    def publish_motion(self):
        self.collect_ik()
        if not self.available() or self.pausing:
            return
        if self.startup_initial is not None:
            self.move_to_reference()
            return
        ready = self.live_motion if self.native_motion else self.joint_stream is not None
        if not ready:
            return
        if time.monotonic() - self.last_ik_feedback >= self.feedback_timeout:
            self.pause('IK feedback expired')
            return
        if self.native_motion:
            self.motion_heartbeat()
        else:
            goal, velocity = self.joint_stream.at(time.monotonic())
            self.send(goal, velocities=velocity)

    def move_to_reference(self):
        elapsed = time.monotonic() - self.startup_started_at
        if elapsed > self.ik.startup_duration + self.startup_timeout:
            self.pause('Start pose was not reached')
        elif self.native_motion:
            self.motion_heartbeat()
        else:
            fraction = np.clip(elapsed / self.ik.startup_duration, 0., 1.)
            progress = fraction**3 * (10 - 15 * fraction + 6 * fraction**2)
            rate = 30 * fraction**2 * (1 - fraction)**2 / self.ik.startup_duration
            delta = self.startup_target - self.startup_initial
            self.send(self.startup_initial + progress * delta, velocities=rate * delta)

    def destroy_node(self):
        self.leader.close()
        self.ik_worker.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--ik-config', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LeaderSource(args.port, args.calibration, args.ik_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
