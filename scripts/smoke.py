"""User-run stages. Each stage writes its own report; no automatic robot enable."""

from pathlib import Path
import argparse
import importlib.util
import json
import math
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def output(args, report):
    directory = Path(
        args.output
        or ROOT
        / "runs"
        / ("manual-" + args.stage + "-" + time.strftime("%Y%m%d-%H%M%S"))
    )
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps(report, indent=2))
    print("Report:", directory / "report.json")


def live_capture(timeout):
    import rclpy
    from rclpy.node import Node
    from robot_server.msg import Capture
    from model_api.capture import decode_capture_packet

    rclpy.init(args=[])
    node = Node("manual_capture_probe")
    values = []

    def receive(message):
        value = decode_capture_packet(bytes(message.packet))
        if (
            value.identity.runtime_instance != message.runtime_instance
            or value.identity.timeline_generation != message.timeline_generation
            or value.identity.observation_sequence != message.observation_sequence
            or value.execution_time_ns != message.execution_time_ns
        ):
            raise ValueError("Capture identity disagreement")
        values.append(value)

    node.create_subscription(Capture, "/robot/capture", receive, 1)
    try:
        deadline = time.monotonic() + timeout
        while not values:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "No capture received. Start server or request robotctl capture in stepped mode."
                )
            rclpy.spin_once(node, timeout_sec=0.1)
        return values[-1]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def fitting(args):
    import numpy as np
    import robot_runtime

    reference = ROOT / "archive/policy_runner/policy_runner/action_timeline.py"
    if not reference.exists():
        reference = ROOT / "src/policy_runner/policy_runner/action_timeline.py"
    spec = importlib.util.spec_from_file_location("legacy_fitting_reference", reference)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    period = 50_000_000
    times = np.arange(81) * 0.05
    actions = np.stack([0.02 * np.sin(times * 0.7), 0.01 * np.cos(times * 0.3)], axis=1)
    lower = np.full(2, -1.0)
    upper = np.full(2, 1.0)
    limits = {"velocity": [1.0] * 2, "acceleration": [2.0] * 2, "jerk": [10.0] * 2}
    tolerance = [0.02] * 2
    old = module.ActionTimeline(actions, 0, 20, lower, upper, limits, tolerance)
    result = robot_runtime.fit_timed_targets(
        actions, 0, period, lower, upper, limits, tolerance
    )
    errors = {
        key: float(np.max(np.abs(np.asarray(result[key]) - getattr(old, key))))
        for key in ("position", "velocity", "acceleration")
    }
    passed = all(value < 1e-6 for value in errors.values())
    output(
        args,
        {
            "stage": "fitting",
            "passed": passed,
            "max_absolute_errors": errors,
            "reference": str(reference),
            "fit_report": {
                key: result[key]
                for key in ("deviation", "budget", "saturation")
                if key in result
            },
            "robot_started": False,
        },
    )
    if not passed:
        raise RuntimeError("Native fitter differs from legacy reference")


def capture(args):
    value = live_capture(args.timeout)
    output(
        args,
        {
            "stage": "capture",
            "identity": value.identity.model_dump(),
            "execution_time_ns": value.execution_time_ns,
            "images": {
                name: {
                    "shape": list(pixels.shape),
                    "dtype": str(pixels.dtype),
                    "metadata": value.observation["images"][name],
                }
                for name, pixels in value.images.items()
            },
            "state": value.observation["state"],
            "motion_sent": False,
        },
    )


def inference(args):
    import numpy as np
    import yaml
    from model_api import Features, RemoteModel
    from model_api.capture import policy_input

    config = yaml.safe_load(Path(args.config).read_text())
    value = live_capture(args.timeout)
    features = Features(
        state_joints=tuple(config["joints"]),
        cameras={
            k: (3, config["resolution"][1], config["resolution"][0])
            for k in config["cameras"]
        },
        action_period=config["period"],
        instruction=config["model"] == "smolvla",
    )
    request = policy_input(
        value,
        features,
        config["cameras"],
        args.instruction,
        "manual-probe",
        "manual-probe",
    )
    with RemoteModel(
        args.url, features, expected_revision=args.revision, timeout=args.timeout
    ) as client:
        prediction, timing = client.predict(request)
    actions = np.asarray(prediction.actions)
    output(
        args,
        {
            "stage": "inference",
            "prediction": prediction.model_dump(),
            "timing": timing,
            "finite": bool(np.isfinite(actions).all()),
            "motion_sent": False,
        },
    )


def record(args):
    import rclpy
    from rclpy.node import Node
    from robot_server.srv import Step, CaptureOnce
    from robot_recorder.srv import Control, Acknowledge

    rclpy.init(args=[])
    node = Node("manual_recording_caller")

    def call(kind, topic, request):
        client = node.create_client(kind, topic)
        try:
            if not client.wait_for_service(timeout_sec=args.timeout):
                raise TimeoutError(topic + " unavailable")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
            if not future.done():
                raise TimeoutError(topic + " timed out")
            return future.result()
        finally:
            node.destroy_client(client)

    def control(operation, arguments):
        reply = call(
            Control,
            "/recording/control",
            Control.Request(operation=operation, arguments_json=json.dumps(arguments)),
        )
        if not reply.applied:
            raise RuntimeError(reply.error)
        return json.loads(reply.result_json)

    def written(identity):
        until = time.monotonic() + args.timeout
        while True:
            reply = call(Acknowledge, "/recording/ack", Acknowledge.Request(**identity))
            if reply.error:
                raise RuntimeError(reply.error)
            if reply.recorded:
                return
            if not reply.pending:
                raise RuntimeError("Recorder did not accept this capture")
            if time.monotonic() >= until:
                raise TimeoutError("Recorder acknowledgement did not complete")
            time.sleep(0.02)

    def event_written(identifier):
        until = time.monotonic() + args.timeout
        while True:
            value = control("event.ack", {"event_id": identifier})
            if not value.get("pending"):
                if not value.get("recorded"):
                    raise RuntimeError(value.get("error", "Event write failed"))
                return
            if time.monotonic() >= until:
                raise TimeoutError("Event acknowledgement did not complete")
            time.sleep(0.02)

    directory = Path(
        args.output
        or ROOT / "runs" / ("manual-record-" + time.strftime("%Y%m%d-%H%M%S"))
    ).resolve()
    args.output = str(directory)
    episode = str(uuid.uuid4())
    captures = []
    started = False
    try:
        control(
            "record.start",
            {
                "directory": str(directory),
                "manifest": {
                    "profile": {"robot_name": args.robot},
                    "action_period": 0.05,
                    "manual_smoke": True,
                },
            },
        )
        started = True
        first = control(
            "episode.start",
            {
                "id": episode,
                "instruction": args.instruction,
                "kind": "demonstration",
                "event_id": str(uuid.uuid4()),
            },
        )
        event_written(first["event_id"])
        # Request and acknowledge initial capture, then each explicitly requested step.
        for index in range(args.steps + 1):
            kind, topic = (
                (CaptureOnce, "/robot/capture_once")
                if index == 0
                else (Step, "/robot/step")
            )
            request = kind.Request(request_key=str(uuid.uuid4()))
            if index:
                request.n = 1
            reply = call(kind, topic, request)
            if not reply.success:
                raise RuntimeError(reply.error)
            identity = {
                key: getattr(reply, key)
                for key in (
                    "runtime_instance",
                    "timeline_generation",
                    "observation_sequence",
                )
            }
            written(identity)
            captures.append({**identity, "execution_time_ns": reply.execution_time_ns})
            print(
                f"capture {index}: written at {reply.execution_time_ns} ns", flush=True
            )
        last = control(
            "episode.finish",
            {
                "id": episode,
                "approved": args.approved,
                "success": None,
                "final_capture": identity,
                "event_id": str(uuid.uuid4()),
            },
        )
        if last.get("ticket"):
            finish_ticket = last["ticket"]
            until = time.monotonic() + args.timeout
            while last.get("pending"):
                if time.monotonic() >= until:
                    raise TimeoutError("Episode finalization timed out")
                last = control("operation.ack", {"ticket": finish_ticket})
                time.sleep(0.02)
        elif last.get("event_id"):
            event_written(last["event_id"])
            last["recorded"] = True
        if last.get("recorded") is not True:
            raise RuntimeError(last.get("error", "Episode finalization failed"))
        stopped = control("record.stop", {})
        ticket = stopped.get("ticket")
        until = time.monotonic() + args.timeout
        while stopped.get("pending"):
            if time.monotonic() >= until:
                raise TimeoutError("Recorder finalization timed out")
            stopped = control("operation.ack", {"ticket": ticket})
            time.sleep(0.02)
        if stopped.get("recorded") is not True:
            raise RuntimeError(stopped.get("error", "Recorder finalization failed"))
        if not stopped.get("result", {}).get("complete"):
            raise RuntimeError("Recorder closed with an incomplete manifest")
        started = False
        output(
            args,
            {
                "stage": "record",
                "directory": str(directory),
                "episode": episode,
                "captures": captures,
                "approved": args.approved,
                "motion_sent": False,
                "finalization": stopped,
            },
        )
    finally:
        if started:
            try:
                control("record.stop", {})
            except Exception as error:
                print(
                    "Recording still needs explicit finalization:",
                    error,
                    file=sys.stderr,
                )
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="stage", required=True)
    for name in ("view", "move", "ownership"):
        p = sub.add_parser(name)
        p.add_argument("--output")
    for name in ("capture", "fitting", "inference", "record"):
        p = sub.add_parser(name)
        p.add_argument("--output")
        p.add_argument("--timeout", type=float, default=30)
        if name == "inference":
            p.add_argument("--config", required=True)
            p.add_argument("--url", default="ws://127.0.0.1:8000")
            p.add_argument("--revision")
            p.add_argument("--instruction", default="Move the cube to the target")
        if name == "record":
            p.add_argument("--steps", type=int, default=10)
            p.add_argument("--robot", default="so101")
            p.add_argument("--approved", action="store_true")
            p.add_argument("--instruction", default="Manual capture smoke")
    args = parser.parse_args()
    if hasattr(args, "timeout") and (
        not math.isfinite(args.timeout) or args.timeout <= 0
    ):
        parser.error("--timeout must be finite and positive")
    if args.stage == "record" and not 1 <= args.steps <= 1000:
        parser.error("--steps must be between 1 and 1000")
    if args.stage in ("view", "move"):
        command = (
            ["ros2", "launch", "robot_bringup", "so101_act.launch.py"]
            if args.stage == "view"
            else ["ros2", "run", "robot_examples", "send_joint", "--finite"]
        )
        process = subprocess.Popen(command)
        try:
            code = process.wait()
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                code = process.wait(timeout=10)
        output(
            args,
            {
                "stage": args.stage,
                "command": command,
                "exit_code": code,
                "verification": "Observe topics and record your findings using the tutorial; process exit does not establish success.",
            },
        )
        return
    if args.stage == "ownership":
        print((ROOT / "docs/tutorials/02-ownership-and-timing.md").read_text())
        return
    globals()[args.stage](args)


if __name__ == "__main__":
    main()
