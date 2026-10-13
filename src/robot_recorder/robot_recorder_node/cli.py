"""Send explicit recording/episode requests; waiting belongs in this caller."""

import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from robot_recorder.srv import Control, Acknowledge


def _identity(text):
    value = json.loads(text)
    if isinstance(value, dict) and "identity" in value:
        value = value["identity"]
    keys = {"runtime_instance", "timeline_generation", "observation_sequence"}
    if not isinstance(value, dict) or set(value) != keys:
        raise ValueError(
            "Capture identity requires runtime_instance, timeline_generation, observation_sequence"
        )
    return value


def main():
    p = argparse.ArgumentParser(description=__doc__, prog="recorderctl")
    sub = p.add_subparsers(dest="command", required=True)
    record = sub.add_parser("record")
    record.add_argument("operation", choices=["start", "stop"])
    record.add_argument("--directory", type=Path)
    record.add_argument("--manifest", type=Path)
    record.add_argument("--capacity", type=int)
    episode = sub.add_parser("episode")
    episode.add_argument("operation", choices=["start", "finish", "suspend"])
    episode.add_argument("--id")
    episode.add_argument("--instruction")
    episode.add_argument(
        "--kind",
        choices=["demonstration", "recovery", "policy", "evaluation"],
        default="demonstration",
    )
    episode.add_argument("--parent")
    episode.add_argument("--approve", action="store_true", default=None)
    episode.add_argument("--success", choices=["true", "false", "unknown"])
    episode.add_argument("--first-capture")
    episode.add_argument("--final-capture")
    episode.add_argument("--event-id")
    status = sub.add_parser("status")
    ack = sub.add_parser("ack")
    ack.add_argument("--capture", required=True)
    call = sub.add_parser("call")
    call.add_argument("operation")
    call.add_argument("--args", default="{}")
    call.add_argument("--args-file", type=Path)
    for parser in [record, episode, status, ack, call]:
        parser.add_argument("--wait", action="store_true")
        parser.add_argument("--timeout", type=float, default=120)
    a = p.parse_args()
    if a.timeout <= 0:
        p.error("Timeout must be positive")
    args = {}
    if a.command == "record":
        operation = "record." + a.operation
        if a.directory:
            args["directory"] = str(a.directory.resolve())
        if a.manifest:
            args["manifest"] = json.loads(a.manifest.read_text())
        if a.capacity is not None:
            args["capacity"] = a.capacity
    elif a.command == "episode":
        operation = "episode." + a.operation
        if a.operation == "start":
            if a.instruction is None:
                p.error("episode start requires --instruction")
            args.update(instruction=a.instruction, kind=a.kind)
        for name in ["id", "parent", "event_id"]:
            if getattr(a, name) is not None:
                args[name] = getattr(a, name)
        if a.approve is not None:
            args["approved"] = a.approve
        if a.success is not None:
            args["success"] = {"true": True, "false": False, "unknown": None}[a.success]
        if a.first_capture:
            args["first_capture"] = _identity(a.first_capture)
        if a.final_capture:
            args["final_capture"] = _identity(a.final_capture)
    elif a.command == "call":
        operation = a.operation
        args = json.loads(a.args_file.read_text() if a.args_file else a.args)
    else:
        operation = "status"
    rclpy.init()
    node = Node("recorder_command_client")
    deadline = time.monotonic() + a.timeout

    def request(client, value):
        while not client.wait_for_service(
            timeout_sec=min(1, max(0.01, deadline - time.monotonic()))
        ):
            if time.monotonic() >= deadline:
                raise TimeoutError("Recorder service unavailable")
        future = client.call_async(value)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=max(0, deadline - time.monotonic())
        )
        if not future.done():
            raise TimeoutError("Recorder request timed out")
        return future.result()

    try:
        if a.command == "ack":
            client = node.create_client(Acknowledge, "/recording/ack")
            identity = _identity(a.capture)
            while True:
                reply = request(client, Acknowledge.Request(**identity))
                result = {
                    "pending": reply.pending,
                    "recorded": reply.recorded,
                    "error": reply.error,
                }
                if not a.wait or not reply.pending:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("Capture acknowledgement timed out")
                time.sleep(0.02)
        else:
            client = node.create_client(Control, "/recording/control")

            def command(op, values):
                reply = request(
                    client,
                    Control.Request(operation=op, arguments_json=json.dumps(values)),
                )
                if not reply.applied:
                    raise RuntimeError(reply.error)
                return json.loads(reply.result_json)

            result = command(operation, args)
            if a.wait:
                while result.get("pending"):
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Recording operation timed out")
                    if "ticket" in result:
                        op, values = "operation.ack", {"ticket": result["ticket"]}
                    elif "event_id" in result:
                        op, values = "event.ack", {"event_id": result["event_id"]}
                    else:
                        raise RuntimeError(
                            "Pending operation omitted its acknowledgement ticket"
                        )
                    time.sleep(0.02)
                    next_result = command(op, values)
                    result = {**result, **next_result}
                if result.get("recorded") is False:
                    raise RuntimeError(
                        result.get("error", "Recording operation failed")
                    )
        print(json.dumps(result, indent=2))
        if result.get("error") or (
            isinstance(result.get("result"), dict)
            and result["result"].get("complete") is False
        ):
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
