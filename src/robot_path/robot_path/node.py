#!/usr/bin/env python3
"""ROS joint-angle producer for a preplanned line, triangle and circle.

The robot server owns execution and backend safety. This node owns path geometry,
IK and user requests to acquire/release a command-source session.
"""

import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray

from robot_description.spec import parol6
from robot_server_node.client import Source
from robot_path.markers import path_markers
from robot_path.planner import PathPlanner
from robot_path.settings import PathSettings


class PathNode(Source):
    def __init__(self, settings, autostart=False):
        self.profile = parol6(settings.gripper, scene="robot_only")
        super().__init__("path", self.profile.spec)
        self.settings = settings
        self.autostart = autostart
        self.autostart_timer = None
        self.plan = None
        self.markers = None
        self.started = False
        self.finished = False
        self.started_at = None
        self.completion_samples = 0
        self.last_completion_status = None
        self.status_received_at = None
        self.status_timeout = self.profile.source_timeout_ns / 1_000_000_000
        self.create_subscription(Bool, "/teleop/engage", self.engage, 10)
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/path/markers", marker_qos
        )
        self.create_timer(1.0, self.publish_markers)

    def receive_status(self, status):
        if status.action_spec_id != self.spec.identity:
            raise ValueError(
                "Robot server contract differs from this robot-only path source"
            )
        self.status_received_at = time.monotonic()
        super().receive_status(status)

    def session_changed(self):
        if self.session and not self.started:
            self.get_logger().info(
                f"Path running; planned duration {self.plan.duration:.1f} s"
            )
        if self.started and not self.session:
            self.finished = True
            self.get_logger().info("Server released the path session")

    def prepare(self):
        if self.status.mode != "Ready" or self.status.source:
            raise RuntimeError(
                "The robot server must be Ready and unowned before planning"
            )
        initial = list(self.status.measured_positions)
        # The gripper is outside this IK chain. Preserve its command rather than
        # treating a soft-limit simulation measurement as a new gripper command.
        initial[-1] = float(
            np.clip(
                self.status.commanded_reference[-1],
                self.profile.spec.lower[-1],
                self.profile.spec.upper[-1],
            )
        )
        planner = PathPlanner(self.settings)
        self.plan = planner.plan(initial, progress=self.get_logger().info)
        self.markers = path_markers(self.plan)
        self.publish_markers()
        self.get_logger().info(self.plan.summary())
        if self.autostart:
            self.get_logger().info("Planning complete; starting the simulation")
            self.autostart_timer = self.create_timer(0.05, self.start_simulation)
        else:
            self.get_logger().info("Use pixi run engage to start; pixi run disengage to pause")
        self.create_timer(
            (self.spec.period_ns if self.native_motion else self.profile.execution_period_ns)
            / 1_000_000_000, self.publish_goal
        )

    def publish_markers(self):
        if self.markers is not None:
            self.marker_publisher.publish(self.markers)

    def start_simulation(self):
        if self.status is None or self.status.mode != "Ready" or self.status.source:
            return
        if time.monotonic() - self.status_received_at > self.status_timeout:
            return
        self.autostart_timer.cancel()
        self.request_motion(True)

    def engage(self, message):
        self.request_motion(message.data)

    def request_motion(self, enabled):
        if not enabled:
            self.release_session()
        elif not self.finished and self.plan is not None and not self.session:
            if (
                self.status_received_at is None
                or time.monotonic() - self.status_received_at > self.status_timeout
            ):
                raise TimeoutError("Robot server feedback is stale")
            difference = (
                np.asarray(self.status.measured_positions[:6]) - self.plan.joints[0, :6]
            )
            # A changed start requires a new plan, rather than a jump to its old start.
            if np.max(np.abs(difference)) > self.settings.start_joint_tolerance:
                raise RuntimeError(
                    "Robot moved since planning; restart the source to plan from its new pose"
                )
            self.control("activate")

    def publish_goal(self):
        if self.finished or self.plan is None or not self.available():
            return
        now = time.monotonic()
        if now - self.status_received_at > self.status_timeout:
            raise TimeoutError("Robot server feedback stopped during the path")
        if not self.started:
            self.started = True
            self.started_at = now
            if self.native_motion:
                self.begin_path(self.plan.times, self.plan.joints)
        elapsed = now - self.started_at
        if self.native_motion:
            self.motion_heartbeat()
            finished = self.motion_complete
        else:
            self.send(self.plan.at(min(elapsed, self.plan.duration)),
                      velocities=self.plan.velocity_at(elapsed))
            finished = elapsed >= self.plan.duration
        if finished:
            # Continue sending the endpoint until feedback reaches it. Serial
            # feedback is step counts; this confirms firmware progress, not encoders.
            error = np.max(np.abs(
                np.asarray(self.status.measured_positions[:6]) - self.plan.joints[-1, :6]
            ))
            if self.status_received_at != self.last_completion_status:
                self.last_completion_status = self.status_received_at
                self.completion_samples = self.completion_samples + 1 if error <= 0.001 else 0
            if self.completion_samples >= 3:
                self.release_session()
                self.get_logger().info("Endpoint reached; server hold requested")
            elif elapsed > self.plan.duration + 5.0:
                raise TimeoutError("Robot did not reach the final joint target within five seconds")

    def release_session(self):
        self.finished = True
        if self.session:
            self.control("pause")


def parse_settings(arguments):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gripper", choices=["pneumatic", "msg_ai_100"], default="pneumatic"
    )
    parser.add_argument("--width", type=float, default=0.18)
    parser.add_argument("--distance", type=float, default=0.30)
    parser.add_argument("--height", type=float, default=0.16)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--tip", default="tool_tcp")
    parser.add_argument("--autostart", choices=["true", "false"], default="false")
    args = parser.parse_args(remove_ros_args(args=["path_node", *arguments])[1:])
    settings = vars(args)
    autostart = settings.pop("autostart") == "true"
    return PathSettings(**settings), autostart


def wait_for_server(node, timeout=30.0):
    deadline = time.monotonic() + timeout
    while node.status is None:
        if time.monotonic() >= deadline:
            raise TimeoutError("No compatible robot server status received")
        rclpy.spin_once(node, timeout_sec=0.1)


def release_before_exit(node):
    node.release_session()
    deadline = time.monotonic() + node.status_timeout
    while node.session and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    arguments = sys.argv[1:]
    settings, autostart = parse_settings(arguments)
    rclpy.init(args=arguments, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = PathNode(settings, autostart=autostart)
        wait_for_server(node)
        node.prepare()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Preserve the original exception, but release control before it escapes.
        if node is not None:
            node.get_logger().error("Path source failed; requesting server pause")
        raise
    finally:
        if node is not None:
            try:
                release_before_exit(node)
            finally:
                node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
