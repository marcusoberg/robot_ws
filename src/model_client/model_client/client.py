"""ROS observations -> model WebSocket -> raw joint target chunks."""

import sys
import argparse
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import yaml
import rclpy
from robot_server.msg import Capture, MotionCommand, CommandResult
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from robot_server_client import Producer
from robot_server_client.activation import Activation
from std_srvs.srv import SetBool
import numpy as np
from robot_inference import Features, PolicyInput, RemoteModel
from .capture import decode_capture


class ModelClient(Producer):
    def __init__(self, source, config, url, instruction):
        if config.get("action_representation", "joint_position") != "joint_position":
            raise ValueError(
                "This profile produces unmapped model outputs, not robot joint targets. "
                "State/action calibration is required before control."
            )
        super().__init__(source)
        self.activation = Activation(self, self.get_logger().info)
        self.create_service(SetBool, "/model/engage", self.engage)
        self.config, self.url, self.instruction = config, url, instruction
        self.features = Features(
            state_joints=tuple(config["joints"]),
            cameras=config.get("camera_shapes") or {
                k: (3, config["resolution"][1], config["resolution"][0])
                for k in config["cameras"]
            },
            action_joints=tuple(config.get("action_joints", config["joints"])),
            state_units=tuple(config.get("state_units", [])),
            action_units=tuple(config.get("action_units", [])),
            action_period=config["period"],
            binary_gripper=config.get("binary_gripper", False),
            action_time_offset_ns=config.get("action_time_offset_ns", 0),
            instruction=config["model"] == "smolvla",
        )
        for prefix, names in (("state", self.features.state_joints), ("action", self.features.action_joints)):
            for suffix, default in (("scale", 1.0), ("offset", 0.0)):
                values = np.asarray(config.get(f"{prefix}_{suffix}", [default]*len(names)), dtype=float)
                if values.shape != (len(names),) or not np.isfinite(values).all() or (suffix == "scale" and np.any(values == 0)):
                    raise ValueError(f"Invalid {prefix}_{suffix}")
                setattr(self, f"{prefix}_{suffix}", values)
        self.motion_joints = self.features.action_joints[:-1] if self.features.binary_gripper else self.features.action_joints
        self.gripper_threshold = config.get("gripper_threshold", 0.5)
        if self.features.binary_gripper and (self.action_scale[-1] != 1 or self.action_offset[-1] != 0 or not 0 < self.gripper_threshold < 1):
            raise ValueError("Invalid binary gripper mapping")
        self.capture = None
        self.remote = None
        self.future = None
        self.previous = None
        self.candidate = None
        self.future_epoch = None
        self._inference_context = str(uuid.uuid4())
        self.epoch = None
        self.worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="model_inference"
        )
        self.create_subscription(Capture, "/robot/capture", self.received, 1)
        self.create_timer(0.02, self.advance)

    def received(self, message):
        try:
            value = decode_capture(message)
            identity = value.identity
            if (
                identity.runtime_instance,
                identity.timeline_generation,
                identity.observation_sequence,
                value.execution_time_ns,
            ) != (
                message.runtime_instance,
                message.timeline_generation,
                message.observation_sequence,
                message.execution_time_ns,
            ):
                raise ValueError("Capture packet and ROS header disagree")
            self.capture = value
        except Exception as error:
            self.get_logger().error("Capture rejected: " + str(error))

    def predict(self, capture, context):
        if self.remote is None:
            self.remote = RemoteModel(
                self.url,
                self.features,
                expected_revision=self.config.get("expected_revision"),
            )
        observation = capture.observation
        if tuple(self.description["joints"]) != self.features.state_joints or tuple(self.description["joints"]) != self.motion_joints:
            raise ValueError("Robot joint order differs from model features")
        state = np.asarray(observation["state"]["position"])
        features = {"observation.state": state * self.state_scale + self.state_offset}
        for feature, camera in self.config["cameras"].items():
            features[feature] = capture.images[camera]
        value = PolicyInput(
            basis=capture.identity,
            execution_time_ns=capture.execution_time_ns,
            features=features,
            capture_metadata={
                "images": {feature: observation["images"][camera]
                           for feature, camera in self.config["cameras"].items()},
                "joint_stamp": observation["state"],
            },
            instruction=self.instruction,
            episode=context,
            context=context,
        )
        return self.remote.predict(value)

    def engage(self, request, response):
        try:
            self.activation.request(request.data)
            response.success, response.message = True, "requested; inspect /robot/result and model logs for completion"
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def advance(self):
        self.activation.tick()
        if self.candidate is not None:
            result = self.results.get(self.candidate)
            if result and result.outcome in (
                CommandResult.ACCEPTED,
                CommandResult.COMPLETED,
            ):
                self.previous = self.candidate
                self.candidate = None
            elif result and result.outcome in (
                CommandResult.REJECTED,
                CommandResult.CANCELED,
            ):
                self.candidate = None
        if self.future and self.future.done():
            future, self.future = self.future, None
            try:
                prediction, timing = future.result()
                if not self.selected or not self.activation.active:
                    return
                current_epoch = (
                    self.state["identity"]["runtime_instance"],
                    self.state["identity"]["timeline_generation"],
                    self.selected["session_generation"],
                )
                if self.future_epoch != current_epoch:
                    return
                identity = self.state["identity"]
                if (
                    prediction.basis.runtime_instance != identity["runtime_instance"]
                    or prediction.basis.timeline_generation
                    != identity["timeline_generation"]
                ):
                    return
                command = self.command(MotionCommand.TIMED_TARGET_CHUNK)
                if command is None:
                    return
                command.target_origin_ns = prediction.execution_time_ns + self.features.action_time_offset_ns
                command.prediction_observation_sequence = (
                    prediction.basis.observation_sequence
                )
                command.sample_period_ns = round(prediction.action_period * 1e9)
                command.lead_ns = round(self.config.get("lead_seconds", 0.15) * 1e9)
                command.join_ns = round(self.config.get("join_seconds", 1.0) * 1e9)
                command.fit_tolerance = [self.config["fit_tolerance"]] * len(self.motion_joints)
                command.trajectory.joint_names = list(self.motion_joints)
                decoded = np.asarray(prediction.actions) * self.action_scale + self.action_offset
                if self.features.binary_gripper:
                    command.gripper_open_samples = (decoded[:, -1] >= self.gripper_threshold).tolist()
                    decoded = decoded[:, :-1]
                for i, position in enumerate(decoded):
                    offset = i * command.sample_period_ns
                    command.trajectory.points.append(
                        JointTrajectoryPoint(
                            positions=position.tolist(),
                            time_from_start=Duration(
                                sec=offset // 1_000_000_000,
                                nanosec=offset % 1_000_000_000,
                            ),
                        )
                    )
                # Only an actually admitted predecessor may become a replacement basis.
                if self.previous:
                    outcome = self.results.get(self.previous)
                    if (
                        outcome
                        and outcome.outcome
                        in (CommandResult.ACCEPTED, CommandResult.COMPLETED)
                        and self.state.get("schedule")
                    ):
                        command.has_predecessor = True
                        command.predecessor_command_id = self.previous
                self.commands.publish(command)
                self.candidate = command.command_id
                self.get_logger().info(
                    f"prediction command={command.command_id} round_trip_ns={timing['round_trip_ns']}"
                )
            except Exception as error:
                self.get_logger().error("Inference stopped: " + str(error))
                if self.remote:
                    self.remote.close()
                self.remote = None
        if (
            self.future is None
            and self.candidate is None
            and self.selected
            and self.activation.active
            and self.capture is not None
            and not (self.state.get("schedule") or {}).get("pending")
        ):
            identity = self.state["identity"]
            epoch = (
                identity["runtime_instance"],
                identity["timeline_generation"],
                self.selected["session_generation"],
            )
            if epoch != self.epoch:
                self._inference_context = str(uuid.uuid4())
                self.previous = None
                self.candidate = None
                self.epoch = epoch
            captured = self.capture
            if (
                captured.identity.runtime_instance == identity["runtime_instance"]
                and captured.identity.timeline_generation
                == identity["timeline_generation"]
            ):
                self.capture = None
                self.future_epoch = epoch
                self.future = self.worker.submit(self.predict, captured, self._inference_context)

    def close(self):
        # Socket timeout bounds shutdown; closing it also interrupts in-flight receive.
        if self.remote:
            self.remote.close()
        self.worker.shutdown(wait=False, cancel_futures=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--url", default="ws://127.0.0.1:8000")
    parser.add_argument("--instruction", default="Move the cube to the target")
    parser.add_argument("--source", default="model")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    config_path = Path(args.config).resolve()
    config = yaml.safe_load(config_path.read_text())
    if (config_path.parent / "manifest.json").is_file():
        from robot_inference.bundle import verify
        config["expected_revision"] = verify(config_path.parent)["revision"]
    rclpy.init()
    node = ModelClient(args.source, config, args.url, args.instruction)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
