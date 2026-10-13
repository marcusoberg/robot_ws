"""Freeze an envelope, then explicitly replay it after reset or selection change."""

import argparse
import sys
import threading
import rclpy
from robot_server.msg import MotionCommand
from robot_server_client import Producer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="boundary_example")
    parser.add_argument(
        "--duplicate",
        action="store_true",
        help="First publish the envelope, then replay the identical request",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Producer(args.source)
    ready = threading.Event()
    frozen = None
    sent = False

    def await_operator():
        try:
            input(
                "After reset, hold, or changing selection in another terminal, press Enter to publish the frozen request: "
            )
            ready.set()
        except EOFError:
            pass

    def publish():
        nonlocal frozen, sent
        if frozen is None:
            frozen = node.command(MotionCommand.JOINT_POSITION)
            if frozen is None:
                return
            frozen.values = node.positions()
            print(
                f"Frozen timeline={frozen.timeline_generation} session={frozen.session_generation} command={frozen.command_id}.",
                flush=True,
            )
            if args.duplicate:
                node.commands.publish(frozen)
                print(
                    "First request published; the next send will have the identical request ID.",
                    flush=True,
                )
            threading.Thread(target=await_operator, daemon=True).start()
        if ready.is_set() and not sent:
            node.commands.publish(frozen)
            sent = True
            print(
                "Frozen envelope published. Inspect /robot/result and confirm no motion restarts.",
                flush=True,
            )

    try:
        node.wait_ready()
        print(
            f"Enable and select {args.source} explicitly in another terminal.",
            flush=True,
        )
        node.create_timer(0.05, publish)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
