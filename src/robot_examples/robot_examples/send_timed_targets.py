"""Publish raw timestamped targets and an optional native replacement."""

import argparse
import math
import sys
import rclpy
from robot_server.msg import MotionCommand, CommandResult
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from robot_server_client import Producer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="timed_example")
    parser.add_argument("--offset", type=float, default=0.01)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="Propose a second chunk after the first is admitted",
    )
    parser.add_argument(
        "--origin-age",
        type=float,
        default=0,
        help="Preserve targets based this many seconds in the past, for late rejection checks",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if (
        not math.isfinite(args.offset)
        or not math.isfinite(args.origin_age)
        or args.origin_age < 0
    ):
        parser.error("Offset must be finite and origin age must be finite/nonnegative")
    rclpy.init()
    node = Producer(args.source)
    previous = None
    finished = False

    def publish():
        nonlocal previous, finished
        if finished:
            return
        if previous is not None:
            result = node.results.get(previous)
            if result is None or result.outcome == CommandResult.QUEUED:
                return
            if result.outcome not in (CommandResult.ACCEPTED, CommandResult.COMPLETED):
                finished = True
                return
            if not args.replace:
                finished = True
                return
            if (node.state.get("schedule") or {}).get("pending"):
                return
        command = node.command(MotionCommand.TIMED_TARGET_CHUNK)
        if command is None:
            return
        q = node.positions()
        command.target_origin_ns = int(node.state["time_ns"]) - round(
            args.origin_age * 1e9
        )
        if command.target_origin_ns < 0:
            return  # Wait until the chosen old origin exists.
        command.prediction_observation_sequence = int(
            node.state["identity"]["observation_sequence"]
        )
        command.sample_period_ns = 50_000_000
        command.lead_ns = 150_000_000
        command.join_ns = 1_000_000_000
        command.fit_tolerance = [0.02] * len(q)
        command.trajectory.joint_names = list(node.description["joints"])
        for index in range(81):
            position = list(q)
            position[0] += args.offset + (0.002 if previous else 0)
            offset = index * command.sample_period_ns
            command.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=position,
                    time_from_start=Duration(
                        sec=offset // 1_000_000_000, nanosec=offset % 1_000_000_000
                    ),
                )
            )
        if previous:
            command.has_predecessor = True
            command.predecessor_command_id = previous
            finished = True
        node.commands.publish(command)
        previous = command.command_id
        print(
            f"Sent command {previous}; inspect native fitting/admission on /robot/result.",
            flush=True,
        )

    try:
        node.wait_ready()
        print(f"Producer ready. Enable, then select {args.source}.", flush=True)
        node.create_timer(0.05, publish)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
