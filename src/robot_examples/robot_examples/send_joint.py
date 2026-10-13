"""Publish one named joint target while selected; C++ generates its motion."""

import sys
import argparse
import rclpy
from robot_server.msg import MotionCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from robot_server_client import Producer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="example")
    parser.add_argument("--joint", type=int, default=0)
    parser.add_argument("--offset", type=float, default=0.03)
    parser.add_argument("--positions", type=float, nargs="+")
    parser.add_argument(
        "--finite",
        action="store_true",
        help="Send one two-second native trajectory, then keep heartbeats live",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Producer(args.source)
    try:
        node.wait_ready()
        q = args.positions or node.positions()
        if len(q) != len(node.description["joints"]):
            raise ValueError("Expected one target per joint")
        if not args.positions:
            if not 0 <= args.joint < len(q):
                raise ValueError("Joint index out of range")
            q[args.joint] += args.offset
        if any(
            not lower <= value <= upper
            for value, lower, upper in zip(
                q, node.description["lower"], node.description["upper"]
            )
        ):
            raise ValueError("Target exceeds joint limits")
        print(
            f"Producer {args.source} ready. In another terminal: robotctl enable; robotctl select {args.source}",
            flush=True,
        )
        sent = False

        def publish():
            nonlocal sent
            if args.finite and sent:
                return
            command = node.command(
                MotionCommand.TRAJECTORY
                if args.finite
                else MotionCommand.JOINT_POSITION
            )
            if command:
                if args.finite:
                    command.trajectory.joint_names = list(node.description["joints"])
                    command.trajectory.points = [
                        JointTrajectoryPoint(
                            positions=node.positions(), time_from_start=Duration()
                        ),
                        JointTrajectoryPoint(
                            positions=q, time_from_start=Duration(sec=2)
                        ),
                    ]
                else:
                    command.values = q
                node.commands.publish(command)
                sent = True

        node.create_timer(0.05, publish)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
