#!/usr/bin/env python3
"""Cartesian pose producer for a line, triangle and circle.

The robot server owns execution and backend safety. This node owns path geometry,
timing and user requests to acquire/release a command-source session.
"""

from robot_core.config import robot_settings
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
from robot_server_client.poses import pose_values
from robot_server_client.source import PoseSource
from robot_path.markers import path_markers
from robot_path.planner import PathPlanner
from robot_path.settings import PathSettings, path_settings


class PathNode(PoseSource):
    def __init__(self, settings, autostart=False):
        self.profile = parol6(settings.gripper, scene="robot_only")
        super().__init__("path", self.profile.spec)
        self.settings = settings
        self.autostart = autostart
        self.autostart_timer = None
        self.plan = None
        self.started = False
        self.finished = False
        self.elapsed = 0.
        self.last_tick = None
        self.waiting_since = None
        self.completion_samples = 0
        self.last_completion_status = None
        self.status_received_at = None
        self.status_timeout = self.profile.source_timeout_ns / 1_000_000_000
        self.create_subscription(Bool, "/teleop/engage", self.engage, robot_settings()["ros"]["command_depth"])
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/path/markers", marker_qos
        )

    def receive_status(self, status):
        super().receive_status(status)
        if self.status is status:
            self.status_received_at = time.monotonic()

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
        if self.status.tcp_link != self.settings.tip:
            raise ValueError(f'Path tip {self.settings.tip} differs from server TCP {self.status.tcp_link}')
        initial_xyz, self.orientation = pose_values(self.status.tcp_pose)
        self.plan = PathPlanner(self.settings).plan(initial_xyz)
        # The route is static. Transient-local durability retains it for late
        # subscribers without serializing thousands of points during motion.
        self.marker_publisher.publish(path_markers(self.plan))
        self.get_logger().info(self.plan.summary())
        if self.autostart:
            self.get_logger().info("Planning complete; starting the simulation")
            self.autostart_timer = self.create_timer(self.spec.period_ns / 1e9, self.start_simulation)
        else:
            self.get_logger().info("Use pixi run engage to start; pixi run disengage to pause")
        self.create_timer(
            self.spec.period_ns / 1_000_000_000, self.publish_goal
        )

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
            measured, _ = pose_values(self.status.tcp_pose)
            if np.linalg.norm(measured - self.plan.xyz[0]) > self.settings.position_tolerance:
                raise RuntimeError(
                    "Robot moved since planning; restart the source to plan from its new pose"
                )
            self.control("activate")

    def publish_goal(self):
        if self.finished or self.plan is None or not self.available():
            return
        now = time.monotonic()
        age = now - self.status_received_at
        if age > self.status_timeout:
            self.get_logger().warning(
                f"Pausing path: last /robot/status callback was {age * 1000:.0f} ms ago "
                f"(deadline {self.status_timeout * 1000:.0f} ms, last acknowledged goal "
                f"{self.status.sequence}, last sent {self.sent_sequence})."
            )
            self.release_session()
            return
        period = self.spec.period_ns / 1e9
        measured, _ = pose_values(self.status.tcp_pose)
        error = np.linalg.norm(measured - self.plan.at(self.elapsed))
        if self.last_tick is None:
            self.last_tick = now
            self.started = True
        # Keep the geometric path close to the measured TCP through corners.
        # Lag slows the route clock; a late callback cannot jump over a segment.
        finished = self.elapsed >= self.plan.duration
        tolerance = self.settings.position_tolerance
        lookahead = max(tolerance, self.settings.linear_rate * self.settings.speed * period)
        if not finished and error <= lookahead:
            self.elapsed = min(self.plan.duration, self.elapsed + min(now - self.last_tick, period))
            self.waiting_since = None
        elif finished and error <= tolerance:
            if self.last_completion_status != self.status.execution_time_ns:
                self.last_completion_status = self.status.execution_time_ns
                self.completion_samples += 1
            if self.completion_samples >= robot_settings()["motion"]["completion_samples"]:
                self.release_session()
                self.get_logger().info("TCP endpoint reached; server hold requested")
                return
        else:
            self.completion_samples = 0
            if self.waiting_since is None:
                self.waiting_since = now
            elif now - self.waiting_since > self.profile.stop_timeout_ns / 1e9:
                self.get_logger().warning(f"Pausing path: TCP is still {error * 1000:.1f} mm from the current target")
                self.release_session()
                return
        self.last_tick = now
        self.send_pose(self.plan.at(self.elapsed), self.orientation, position_only=True)

    def release_session(self):
        if self.finished:
            return
        self.finished = True
        if self.session:
            self.control("pause")


def parse_settings(arguments):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gripper", choices=["pneumatic", "msg_ai_100"], default=path_settings()["path"]["gripper"]
    )
    parser.add_argument("--width", type=float, default=path_settings()["path"]["width"])
    parser.add_argument("--distance", type=float, default=path_settings()["path"]["distance"])
    parser.add_argument("--height", type=float, default=path_settings()["path"]["height"])
    parser.add_argument("--speed", type=float, default=path_settings()["path"]["speed"])
    parser.add_argument("--tip", default=path_settings()["path"]["tip"])
    parser.add_argument("--autostart", choices=["true", "false"], default=str(path_settings()["path_planner"]["autostart"]).lower())
    args = parser.parse_args(remove_ros_args(args=["path_node", *arguments])[1:])
    settings = vars(args)
    autostart = settings.pop("autostart") == "true"
    return PathSettings.configured(**settings), autostart


def wait_for_server(node):
    timeout = path_settings()["path_planner"]["discovery_timeout"]
    deadline = time.monotonic() + timeout
    while node.status is None:
        if time.monotonic() >= deadline:
            raise TimeoutError("No compatible robot server status received")
        rclpy.spin_once(node, timeout_sec=robot_settings()["ros"]["control_poll_period"])


def release_before_exit(node):
    node.release_session()
    deadline = time.monotonic() + node.status_timeout
    while node.session and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=robot_settings()["motion"]["status_period"])
    if node.session:
        node.get_logger().warning("No server confirmation of path session release before exit")


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
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
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
