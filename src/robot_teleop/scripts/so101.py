#!/usr/bin/env python3
"""Publish SO101 leader motion as Cartesian targets relative to the measured follower TCP."""

import argparse
from pathlib import Path
import time

import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from std_msgs.msg import Bool

from robot_core.config import robot_settings
from robot_description.spec import load_profile
from robot_server_client.poses import pose_values
from robot_server_client.source import PoseSource
from robot_teleop.config import tracking_profile
from robot_teleop.leader import LeaderFeedbackTimeout, SerialLeader
from robot_teleop.leader_pose import LeaderPose


class LeaderSource(PoseSource):
    def __init__(self, port, calibration, profile_name):
        config = tracking_profile(profile_name)
        self.mapping = LeaderPose(config)
        self.profile = load_profile(**config['follower'])
        super().__init__('so101_source', self.profile.spec)
        self.leader = SerialLeader(port, calibration, self.mapping.spec)
        self.leader.start_reader()
        self.feedback_timeout = self.profile.source_timeout_ns / 1e9
        self.start_pose = config['start_pose']
        self.prepared_episode = None
        self.waiting_for_move = False
        self.aligned = False
        self.want_control = False
        self.pausing = False
        self.last_sample = 0
        self.status_at = 0.
        self.goal_pose = self.create_publisher(PoseStamped, '/teleop/target', robot_settings()['ros']['latest_depth'])
        self.create_subscription(Bool, '/teleop/engage', self.engage, robot_settings()['ros']['latest_depth'])
        self.create_timer(self.spec.period_ns / 1e9, self.sample)
        self.get_logger().info('Paused. Use pixi run engage / pixi run disengage.')

    def receive_status(self, status):
        super().receive_status(status)
        if self.status is status:
            self.status_at = time.monotonic()

    def engage(self, message):
        self.want_control = message.data
        if not message.data:
            if self.session:
                self.pause('Disengaged')
            return
        if self.session or self.pending_commands:
            return
        try:
            self.fresh_sample()
        except LeaderFeedbackTimeout as error:
            self.get_logger().warning(str(error))
            return
        if self.status is None or time.monotonic() - self.status_at > self.feedback_timeout:
            self.get_logger().warning('Waiting for fresh robot status')
            return
        episode = self.status.instance, self.status.episode
        self.waiting_for_move = self.start_pose and self.prepared_episode != episode
        self.control('activate_teleop' if self.waiting_for_move else 'activate')

    def fresh_sample(self):
        raw, acquired = self.leader.latest_raw()
        if time.monotonic() - acquired / 1e9 >= self.feedback_timeout:
            raise LeaderFeedbackTimeout('Leader feedback expired')
        return raw, acquired

    def pause(self, reason):
        if self.session and not self.pausing:
            self.pausing = True
            self.get_logger().info(f'{reason}; requesting hold')
            self.control('pause')

    def session_changed(self):
        self.aligned = False
        self.pausing = False
        self.last_sample = 0
        self.session_started = time.monotonic()
        self.get_logger().info(f'{self.status.mode}: {self.status.reason or self.status.source}')

    def sample(self):
        if not self.available() or self.pausing:
            return
        if not self.want_control or time.monotonic() - self.status_at > self.feedback_timeout:
            self.pause('Disengaged or robot status expired')
            return
        try:
            raw, acquired = self.fresh_sample()
        except LeaderFeedbackTimeout as error:
            self.pause(str(error))
            return
        if self.waiting_for_move:
            if not self.status.motion_complete:
                deadline = robot_settings()['motion']['duration'] + self.profile.stop_timeout_ns / 1e9
                if time.monotonic() - self.session_started > deadline:
                    self.pause('Server did not confirm the teleop start move')
                else:
                    self.renew()
                return
            self.prepared_episode = self.status.instance, self.status.episode
            self.waiting_for_move = False
        if acquired == self.last_sample:
            return
        self.last_sample = acquired
        leader, gripper = self.mapping.read(raw, self.leader.calibration)
        if not self.aligned:
            xyz, quat = pose_values(self.status.tcp_pose)
            tcp = pin.SE3(pin.Quaternion(*quat[[3, 0, 1, 2]]).matrix(), xyz)
            self.mapping.align(leader, tcp)
            self.aligned = True
            self.get_logger().info(f'Leader aligned to measured {self.status.tcp_link}; Cartesian tracking active')
        target = self.mapping.target(leader)
        sample_age = Duration(nanoseconds=time.monotonic_ns() - acquired)
        stamp = (self.get_clock().now() - sample_age).to_msg()
        sent = self.send_pose(target.translation, pin.Quaternion(target.rotation).coeffs(), gripper=gripper, stamp=stamp)
        if sent is not None:
            self.goal_pose.publish(sent)

    def destroy_node(self):
        if self.session and rclpy.ok() and not self.pausing:
            self.control('pause')
        self.leader.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--profile', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LeaderSource(args.port, args.calibration, args.profile)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
