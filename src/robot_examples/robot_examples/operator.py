"""Explicit operator actions. Queued control never hides a simulation step."""

import sys
import argparse
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from robot_server.msg import Observation
from robot_server.srv import Control, Step, Reset, CaptureOnce

ACTIONS = {
    "enable": Control.Request.ENABLE,
    "disable": Control.Request.DISABLE,
    "stop": Control.Request.STOP,
    "hold": Control.Request.HOLD,
    "standby": Control.Request.STANDBY,
    "open-gripper": Control.Request.OPEN_GRIPPER,
    "close-gripper": Control.Request.CLOSE_GRIPPER,
    "select": Control.Request.SELECT,
    "takeover": Control.Request.TAKEOVER,
    "release": Control.Request.RELEASE,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action", choices=(*ACTIONS, "status", "sources", "step", "reset", "capture")
    )
    parser.add_argument("source", nargs="?")
    parser.add_argument("--instance")
    parser.add_argument("--n", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=10)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Node("robot_operator")
    states = []
    node.create_subscription(
        Observation,
        "/robot/state",
        lambda message: states.append(json.loads(message.record_json)),
        1,
    )
    try:
        until = time.monotonic() + args.timeout
        while not states:
            if time.monotonic() >= until:
                raise TimeoutError("Robot state unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)
        state = states[-1]
        if args.action in ("status", "sources"):
            print(
                json.dumps(
                    state
                    if args.action == "status"
                    else state.get("available_sources", []),
                    indent=2,
                )
            )
            return
        if args.action == "step":
            kind, topic, request = (
                Step,
                "/robot/step",
                Step.Request(n=args.n, request_key=str(uuid.uuid4())),
            )
        elif args.action == "reset":
            kind, topic, request = (
                Reset,
                "/robot/reset",
                Reset.Request(request_key=str(uuid.uuid4())),
            )
        elif args.action == "capture":
            kind, topic, request = (
                CaptureOnce,
                "/robot/capture_once",
                CaptureOnce.Request(request_key=str(uuid.uuid4())),
            )
        else:
            source, instance = args.source or "", args.instance or ""
            if args.action in ("select", "takeover"):
                matches = [
                    v
                    for v in state.get("available_sources", [])
                    if v["source"] == source
                    and (not instance or v["source_instance"] == instance)
                ]
                if len(matches) != 1:
                    raise ValueError(
                        "Source must match one live producer. Run robotctl sources; use --instance if ambiguous."
                    )
                instance = matches[0]["source_instance"]
            kind, topic = Control, "/robot/control"
            request = Control.Request(
                action=ACTIONS[args.action],
                source=source,
                source_instance=instance,
                request_key=str(uuid.uuid4()),
            )
        client = node.create_client(kind, topic)
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise TimeoutError(topic + " unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            raise TimeoutError(topic + " did not reply")
        result = future.result()
        print(json.dumps(message_to_ordereddict(result), indent=2))
        if hasattr(result, "success") and not result.success:
            raise RuntimeError(result.error)
        if hasattr(result, "queued") and not result.queued:
            raise RuntimeError(result.message)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
