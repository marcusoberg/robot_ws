"""The ROS robot owner. Native execution owns timing, authority and interpolation."""

from collections import OrderedDict
from dataclasses import dataclass, replace
import json
import hashlib
import math
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time
from uuid import UUID, uuid4
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.task import Future
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage

from robot_runtime import (
    Configuration, ObservationIdentity, Pose, Robot, TimedJointTargets,
    Trajectory, Waypoint,
)
from robot_server.msg import Capture, CommandResult, Event, Heartbeat, MotionCommand, Observation
from robot_server.srv import CaptureOnce, Control, Reset, Step
from robot_server_node import conversion
from robot_server_node.qos import CAPTURES, DESCRIPTION, EVENTS, LATEST, STATE
from robot_server_node.visualization import SceneFrames


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if hasattr(value, "items"):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def dumps(value):
    return json.dumps(plain(value), allow_nan=False)


@dataclass
class Source:
    handle: object
    name: str
    instance: str
    heartbeat_sequence: int = 0
    command_sequence: int = 0
    seen: float = 0.0


@dataclass
class Pending:
    operation: object
    source: Source | None
    request_id: int
    command_id: int
    name: str
    session_generation: int
    key: tuple | None = None
    last_outcome: str = "queued"


class RobotServer(Node):
    def __init__(self):
        super().__init__("robot_server")
        deployment = self.declare_parameter("deployment", "").value
        if not deployment:
            raise ValueError("deployment must name a robot configuration YAML")
        self.configuration = Configuration.from_yaml(Path(deployment))
        self.backend = self.declare_parameter("backend", "mujoco").value
        self.mode = self.declare_parameter("mode", "realtime").value
        self.action_period = float(self.declare_parameter("action_period", 0.05).value)
        self.publish_period = float(self.declare_parameter("publish_period", 0.05).value)
        if not math.isfinite(self.action_period) or self.action_period <= 0:
            raise ValueError("action_period must be finite and positive")
        if not math.isfinite(self.publish_period) or self.publish_period <= 0:
            raise ValueError("publish_period must be finite and positive")
        self.robot = Robot.local(
            self.configuration, backend=self.backend, mode=self.mode,
            action_period=self.action_period if self.mode == "stepped" else None,
        )
        self.description = dict(self.robot.description)
        self.initial = self.robot.observe()
        self.runtime_instance = self.initial.identity.runtime_instance
        self.host_clock = "robot_server.host.monotonic:" + self.runtime_instance
        self.callbacks = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(Observation, "/robot/state", STATE)
        self.capture_pub = self.create_publisher(Capture, "/robot/capture", CAPTURES)
        self.event_pub = self.create_publisher(Event, "/robot/events", EVENTS)
        self.result_pub = self.create_publisher(CommandResult, "/robot/result", EVENTS)
        self.joints_pub = self.create_publisher(JointState, "/joint_states", LATEST)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", LATEST)
        self.static_tf_pub = self.create_publisher(TFMessage, "/tf_static", DESCRIPTION)
        self.reference_pub = self.create_publisher(JointState, "/robot/reference", LATEST)
        self.tracking_pub = self.create_publisher(Float64MultiArray, "/robot/tracking_error", LATEST)
        self.ownership_pub = self.create_publisher(String, "/robot/ownership", LATEST)
        self.timing_pub = self.create_publisher(Float64MultiArray, "/robot/timing", LATEST)
        self.description_pub = self.create_publisher(String, "/robot/description", DESCRIPTION)
        self.urdf_pub = self.create_publisher(String, "/robot/urdf", DESCRIPTION)
        self.clock_pub = self.create_publisher(Clock, "/clock", LATEST) if self.backend == "mujoco" else None
        self.camera_publishers = {}
        for camera in self.configuration.cameras:
            for suffix in ("rgb", "depth") if camera.depth else ("rgb",):
                name = camera.name + "/" + suffix
                topic = "/robot/cameras/" + name
                self.camera_publishers[name] = (
                    self.create_publisher(CompressedImage if suffix == "rgb" else Image, topic, LATEST),
                    self.create_publisher(CameraInfo, topic + "/camera_info", LATEST),
                )
        self.create_subscription(MotionCommand, "/robot/command", self._command, 16, callback_group=self.callbacks)
        self.create_subscription(Heartbeat, "/robot/heartbeat", self._heartbeat, LATEST, callback_group=self.callbacks)
        self.create_service(Control, "/robot/control", self._control, callback_group=self.callbacks)
        self.create_service(Step, "/robot/step", self._step, callback_group=self.callbacks)
        self.create_service(Reset, "/robot/reset", self._reset, callback_group=self.callbacks)
        self.create_service(CaptureOnce, "/robot/capture_once", self._capture_once, callback_group=self.callbacks)
        self.jobs = Queue(256)
        self.heartbeat_jobs = Queue(128)
        self.urgent = Queue(8)
        self.closed = threading.Event()
        self.lock = threading.RLock()
        self.sources = {}
        self.selected = None
        self.session = None
        self.input_blocked = None
        self.pending = []
        self.predecessors = OrderedDict()
        self.bases = OrderedDict()
        self.command_replies = OrderedDict()
        self.command_fingerprints = OrderedDict()
        self.control_replies = OrderedDict()
        self.service_replies = OrderedDict()
        self.next_request = 0
        self.delivery_sequence = 0
        self.last_capture = None
        self.camera_error = None
        self.server_error = None
        self.scene_frames = SceneFrames(self.configuration, self.description) if self.backend == "mujoco" else None
        self.static_frames = self._fixed_frames()
        self._publish_description()
        self._publish_static(self.initial)
        self._publish_state(self.initial)
        self.worker = threading.Thread(target=self._work, name="robot-ros-adapter", daemon=False)
        self.worker.start()
        self.get_logger().info(f"Robot server ready ({self.backend}, {self.mode}); DISABLED and no selected source")

    def _fixed_frames(self):
        tree = ET.parse(self.configuration.robot.description)
        reachable = {self.description["base"]}
        edges = [(j.find("parent").get("link"), j.find("child").get("link"))
                 for j in tree.getroot().findall("joint") if j.get("type") == "fixed"]
        while added := {child for parent, child in edges if parent in reachable} - reachable:
            reachable.update(added)
        return reachable - {self.description["base"]}

    def _publish_description(self):
        description = {**self.description, "server": {"mode": self.mode, "backend": self.backend, "action_period": self.action_period}}
        self.description_pub.publish(String(data=dumps(description)))
        self.urdf_pub.publish(String(data=self.configuration.robot.description.read_text()))

    def _publish_static(self, value):
        if value.joints is None:
            return
        poses = self.robot.frames(value)
        self.static_tf_pub.publish(TFMessage(transforms=[
            conversion.transform(self.description["base"], name, poses[name], 0)
            for name in sorted(self.static_frames)
        ]))

    def _cache_basis(self, value):
        # Native snapshot handles remain usable; image arrays do not accumulate.
        key = (value.epoch, value.sequence)
        if key not in self.bases:
            self.bases[key] = (replace(value, images={}), time.monotonic())
        while len(self.bases) > 1024:
            self.bases.popitem(last=False)

    def _source_metadata(self, source):
        generation = self.session.generation if source is self.selected and self.session is not None else 0
        return {"source": source.name, "source_instance": source.instance, "session_generation": generation,
                "source_id": source.handle.source_id, "last_seen_seconds": max(0.0, time.monotonic() - source.seen) if source.seen else None}

    def _publish_state(self, value):
        self.current_observation = value
        self._cache_basis(value)
        record = value.to_record()
        record["selected_source"] = self._source_metadata(self.selected) if self.selected else None
        record["available_sources"] = [self._source_metadata(source) for source in self.sources.values()
                                       if source is self.selected or time.monotonic() - source.seen < 2.0]
        record["server"] = {"driver_mode": self.mode, "camera_error": self.camera_error, "error": self.server_error,
                            "pending_requests": len(self.pending)}
        identity = value.identity
        self.state_pub.publish(Observation(
            runtime_instance=identity.runtime_instance, timeline_generation=value.epoch,
            observation_sequence=value.sequence, execution_time_ns=int(record["time_ns"]), record_json=dumps(record),
        ))
        if value.joints is not None:
            self.joints_pub.publish(conversion.joints(value, self.description))
            self.tf_pub.publish(conversion.frames(self.robot, value, exclude=self.static_frames))
            reference = conversion.joints(value, self.description)
            reference.position = list(value.reference["position"])
            reference.velocity = list(value.reference["velocity"])
            self.reference_pub.publish(reference)
            self.tracking_pub.publish(Float64MultiArray(data=value.tracking_error.tolist()))
        self.ownership_pub.publish(String(data=dumps({"mode": value.mode.value, "native_owner": value.ownership.source,
                                                     "selected_source": record["selected_source"], "generation": value.ownership.session_generation})))
        self.timing_pub.publish(Float64MultiArray(data=[value.timing.control_interval, value.timing.host_interval,
                                                      value.timing.cycle_duration, float(value.timing.coalesced_observations)]))
        if self.clock_pub is not None:
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(round(value.time * 1e9), 1_000_000_000)
            self.clock_pub.publish(clock)

    def _publish_capture(self, value, *, repeat=False):
        self._cache_basis(value)
        identity = value.identity
        if self.last_capture is not None and (
            self.last_capture.timeline_generation, self.last_capture.observation_sequence
        ) == (value.epoch, value.sequence):
            if repeat:
                self.capture_pub.publish(self.last_capture)
            return identity
        self.delivery_sequence += 1
        message = conversion.capture(value, self.delivery_sequence, self.host_clock)
        self.capture_pub.publish(message)
        self.last_capture = message
        camera_transforms = []
        base_from_world = self.scene_frames.base_from_world(value) if self.scene_frames else np.eye(4)
        from scipy.spatial.transform import Rotation
        for name, sample in value.images.items():
            if name not in self.camera_publishers:
                # External cameras may be configured as named SDK sensor planes.
                topic = "/robot/cameras/" + name
                self.camera_publishers[name] = (
                    self.create_publisher(CompressedImage if sample.pixels.ndim == 3 else Image, topic, LATEST),
                    self.create_publisher(CameraInfo, topic + "/camera_info", LATEST),
                )
            base_from_camera = base_from_world @ sample.world_from_camera
            base_from_camera[:3, :3] = base_from_camera[:3, :3] @ np.diag([1, -1, -1])
            xyzw = Rotation.from_matrix(base_from_camera[:3, :3]).as_quat()
            pose = Pose(base_from_camera[:3, 3], (xyzw[3], *xyzw[:3]))
            camera_transforms.append(conversion.transform(self.description["base"], sample.frame + "_optical", pose, round(value.time * 1e9)))
            stamp = sample.stamp.source_time_ns if sample.stamp.source_time_ns is not None else round(value.time * 1e9)
            publisher, calibration = self.camera_publishers[name]
            publisher.publish(conversion.compressed_image(sample, stamp) if sample.pixels.ndim == 3 else conversion.image(sample, stamp))
            calibration.publish(conversion.calibration(sample, stamp))
        if camera_transforms:
            self.tf_pub.publish(TFMessage(transforms=camera_transforms))
        return identity

    def _event(self, kind, details=None, basis=None):
        basis = basis or getattr(self, "current_observation", self.initial)
        self.event_pub.publish(Event(
            event_id=uuid4().hex, kind=kind, host_ns=time.monotonic_ns(), host_clock=self.host_clock,
            has_basis=True, runtime_instance=basis.identity.runtime_instance,
            timeline_generation=basis.epoch, observation_sequence=basis.sequence,
            details_json=dumps(details or {}),
        ))

    def _reply(self, source, request_id, command_id, name, outcome, reason="none", generation=0, native_id=0, key=None):
        value = CommandResult(
            source=source.name if source else "", source_instance=source.instance if source else "",
            runtime_instance=self.runtime_instance,
            timeline_generation=key[3] if key is not None else getattr(self, "current_observation", self.initial).epoch,
            session_generation=generation, request_id=request_id, command_id=command_id, native_command_id=native_id,
            operation=name, outcome=getattr(CommandResult, outcome.upper()), reason=str(reason),
        )
        self.result_pub.publish(value)
        if key is not None:
            self.command_replies[key] = value
            self.command_replies.move_to_end(key)
            while len(self.command_replies) > 512:
                removed, _ = self.command_replies.popitem(last=False)
                self.command_fingerprints.pop(removed, None)
        return value

    def _name(self, name, instance):
        if not name or len(name) > 128 or any(not (c.isalnum() or c in "_.-") for c in name):
            raise ValueError("source must be a short logical name")
        UUID(instance)
        return name, instance

    def _source(self, name, instance, *, create=False):
        key = self._name(name, instance)
        if key not in self.sources:
            if not create:
                raise ValueError("source is not registered; publish a heartbeat first")
            if len(self.sources) >= 64:
                raise ValueError("source registry is full; restart with fewer actors")
            self.sources[key] = Source(self.robot.source(name, instance), name, instance)
        return self.sources[key]

    def _basis(self, message, *, heartbeat=False):
        if message.runtime_instance != self.runtime_instance:
            raise ValueError("input belongs to another runtime instance")
        current = self.robot.observe()
        if message.timeline_generation != current.epoch:
            raise ValueError("input belongs to an old timeline")
        if not 0 < message.ttl_ns <= 1_000_000_000:
            raise ValueError("ttl_ns must be positive and at most one second")
        cached = self.bases.get((message.timeline_generation, message.basis_observation_sequence))
        if cached is None:
            raise ValueError("input basis was not published by this server")
        basis, published = cached
        # Renewals are fresh host-clock calls. A paused stepped snapshot also
        # remains current; motion expiry there belongs to execution time.
        elapsed = current.time - basis.time if self.mode == "stepped" else time.monotonic() - published
        if not heartbeat and elapsed >= message.ttl_ns / 1e9:
            raise ValueError("input observation expired")
        return basis

    def _command(self, message):
        try:
            self.jobs.put_nowait(("motion", (message, time.monotonic())))
        except Full:
            self._reply(Source(None, message.source, message.source_instance), 0, message.command_id,
                        "motion", "rejected", "robot input queue is full", message.session_generation)

    def _heartbeat(self, message):
        try:
            self.heartbeat_jobs.put_nowait(("heartbeat", (message, time.monotonic())))
        except Full:
            self._event("input.rejected", {"reason": "robot input queue is full", "source": message.source})

    def _request_key(self, key):
        if not key:
            raise ValueError("request_key requires a caller UUID for idempotent retries")
        UUID(key)
        return key

    def _control(self, request, response):
        with self.lock:
            try:
                if self.closed.is_set():
                    raise ValueError("server is closing")
                key = self._request_key(request.request_key)
                if key in self.control_replies:
                    saved, payload = self.control_replies[key]
                    current = (request.action, request.source, request.source_instance)
                    if current != payload:
                        raise ValueError("request_key was reused with different control fields")
                    for name, value in saved.items():
                        setattr(response, name, value)
                    return response
                if request.action > Control.Request.STATUS:
                    raise ValueError("unknown control action")
                self.next_request += 1
                response.request_id = self.next_request
                response.session_generation = self.session.generation if self.session is not None else 0
                job = ("control", (response.request_id, request.action, request.source, request.source_instance))
                queue = self.urgent if request.action in (Control.Request.STOP, Control.Request.DISABLE, Control.Request.HOLD) else self.jobs
                queue.put_nowait(job)
                response.queued = True
                response.message = "queued; observe /robot/result for applied outcome"
                saved = {name: getattr(response, name) for name in ("queued", "request_id", "session_generation", "message")}
                self.control_replies[key] = (saved, (request.action, request.source, request.source_instance))
                while len(self.control_replies) > 512:
                    self.control_replies.popitem(last=False)
            except (ValueError, Full) as error:
                response.queued = False
                response.message = str(error)
        return response

    async def _owner_request(self, name, request, response):
        try:
            key = (name, self._request_key(request.request_key))
            arguments = int(request.n) if name == "step" else None
            with self.lock:
                if self.closed.is_set():
                    raise ValueError("server is closing")
                if key in self.service_replies:
                    future, original = self.service_replies[key]
                    if original != arguments:
                        raise ValueError("request_key was reused with different operation fields")
                else:
                    future = Future(executor=self.executor)
                    self.service_replies[key] = (future, arguments)
                    try:
                        self.jobs.put_nowait((name, (arguments, future)))
                    except Full:
                        self.service_replies.pop(key)
                        raise ValueError("robot input queue is full")
                    while len(self.service_replies) > 512:
                        oldest, (saved, _) = next(iter(self.service_replies.items()))
                        if not saved.done():
                            break
                        self.service_replies.pop(oldest)
            result = await future
            for field, value in result.items():
                setattr(response, field, value)
        except ValueError as error:
            response.success = False
            response.error = str(error)
        return response

    async def _step(self, request, response):
        return await self._owner_request("step", request, response)

    async def _reset(self, request, response):
        return await self._owner_request("reset", request, response)

    async def _capture_once(self, request, response):
        return await self._owner_request("capture", request, response)

    def _handle_heartbeat(self, message, received):
        if time.monotonic() - received >= message.ttl_ns / 1e9:
            raise ValueError("heartbeat expired in the adapter queue")
        basis = self._basis(message, heartbeat=True)
        source = self._source(message.source, message.source_instance, create=True)
        if message.sequence == 0:
            raise ValueError("heartbeat producer sequence must be positive")
        if message.sequence <= source.heartbeat_sequence:
            return
        source.heartbeat_sequence = message.sequence
        source.seen = time.monotonic()
        if source is self.selected and self.session is not None:
            if message.session_generation != self.session.generation:
                return
            if not source.handle.renew(self.session, observation=basis, valid_for=message.ttl_ns / 1e9):
                self._event("authority.renewal_rejected", {"source": source.name}, basis)

    def _ordered(self, names, values):
        names, values = tuple(names), tuple(values)
        model = tuple(self.description["joints"])
        if len(names) != len(values) or len(names) != len(model) or len(set(names)) != len(names) or set(names) != set(model):
            raise ValueError("named coordinates must match every configured joint exactly once")
        if any(not math.isfinite(value) for value in values):
            raise ValueError("joint values must be finite")
        named = dict(zip(names, values))
        return tuple(named[name] for name in model)

    def _path(self, trajectory, *, raw_targets=False):
        if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
            raise ValueError("trajectory header stamps are unsupported; use runtime-clock target_origin_ns")
        if not 1 <= len(trajectory.points) <= 512:
            raise ValueError("path requires between one and 512 points")
        output = []
        previous = -1
        for point in trajectory.points:
            if point.time_from_start.nanosec >= 1_000_000_000 or point.time_from_start.sec < 0:
                raise ValueError("invalid relative point time")
            ns = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            if ns <= previous:
                raise ValueError("point times must strictly increase")
            previous = ns
            if point.effort or point.accelerations or (raw_targets and point.velocities):
                raise ValueError("raw targets contain positions only; finite paths do not accept acceleration or effort")
            q = self._ordered(trajectory.joint_names, point.positions)
            velocity = self._ordered(trajectory.joint_names, point.velocities) if point.velocities else None
            output.append((ns, q, velocity))
        return output

    def _handle_motion(self, message, received):
        source = Source(None, message.source, message.source_instance)
        key = (message.source, message.source_instance, message.runtime_instance, message.timeline_generation,
               message.session_generation, message.command_id)
        try:
            serialized = serialize_message(message)
            if len(serialized) > 2 * 1024 * 1024:
                raise ValueError("motion packet exceeds the two MiB ingress budget")
            fingerprint = hashlib.sha256(serialized).digest()
            if key in self.command_replies:
                if self.command_fingerprints.get(key) != fingerprint:
                    self._reply(source, 0, message.command_id, "motion", "rejected",
                                "command_id was reused with different fields", message.session_generation)
                    return
                self.result_pub.publish(self.command_replies[key])
                return
            self.command_fingerprints[key] = fingerprint
            if time.monotonic() - received >= message.ttl_ns / 1e9:
                raise ValueError("motion expired in the adapter queue")
            basis = self._basis(message)
            source = self._source(message.source, message.source_instance)
            if source is not self.selected or self.session is None or not self.session.valid or message.session_generation != self.session.generation:
                raise ValueError("source does not own the selected native session")
            if message.command_id == 0 or message.sequence <= source.command_sequence:
                raise ValueError("command_id and producer sequence must be positive and increasing")
            source.command_sequence = message.sequence
            if not message.deadman or message.clutch:
                if self.input_blocked is not True:
                    operation = self.session.pause()
                    self.pending.append(Pending(operation, source, 0, 0, "pause", self.session.generation))
                    self._event("motion.paused", {"source": source.name, "command_id": message.command_id}, basis)
                    self.input_blocked = True
                self._reply(source, 0, message.command_id, "motion", "canceled", "deadman_or_clutch", self.session.generation, key=key)
                return
            kwargs = {"observation": basis, "valid_for": message.ttl_ns / 1e9}
            if message.has_gripper_command:
                kwargs["gripper_open"] = message.gripper_open
            if message.kind == MotionCommand.JOINT_POSITION:
                operation = self.session.target_joints(self._ordered(message.joint_names, message.values), **kwargs)
            elif message.kind == MotionCommand.JOINT_VELOCITY:
                operation = self.session.joint_velocity(self._ordered(message.joint_names, message.values), **kwargs)
            elif message.kind == MotionCommand.BASE_POSE:
                if len(message.auxiliary_names) != len(message.auxiliary_positions) or len(set(message.auxiliary_names)) != len(message.auxiliary_names):
                    raise ValueError("auxiliary names and positions disagree")
                p, q = message.pose.position, message.pose.orientation
                operation = self.session.target_pose((p.x, p.y, p.z), (q.w, q.x, q.y, q.z), frame=message.frame_id,
                    auxiliary_positions=dict(zip(message.auxiliary_names, message.auxiliary_positions)), **kwargs)
            elif message.kind == MotionCommand.BASE_TWIST:
                t = message.twist
                operation = self.session.twist((t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z), frame=message.frame_id, **kwargs)
            elif message.kind == MotionCommand.TRAJECTORY:
                points = self._path(message.trajectory)
                operation = self.session.trajectory(Trajectory(tuple(Waypoint(ns / 1e9, q, velocity) for ns, q, velocity in points)), **kwargs)
            elif message.kind == MotionCommand.TIMED_TARGET_CHUNK:
                points = self._path(message.trajectory, raw_targets=True)
                if message.target_origin_ns < 0 or message.sample_period_ns <= 0 or message.lead_ns < 0 or message.join_ns <= 0:
                    raise ValueError("timed chunk origin/period/lead/join are invalid")
                if any(ns != i * message.sample_period_ns for i, (ns, _, _) in enumerate(points)):
                    raise ValueError("target point offsets must preserve the declared sample period")
                if (message.timeline_generation, message.prediction_observation_sequence) not in self.bases:
                    raise ValueError("original prediction basis was not published by this server")
                prediction_basis = ObservationIdentity(self.runtime_instance, message.timeline_generation, message.prediction_observation_sequence)
                predecessor = None
                if message.has_predecessor:
                    predecessor = self.predecessors.get((source.name, source.instance, self.session.generation, message.predecessor_command_id))
                    if predecessor is None:
                        raise ValueError("predecessor is not an admitted command in this session")
                tolerance = tuple(message.fit_tolerance)
                if len(tolerance) not in (1, len(self.description["joints"])) or any(not math.isfinite(v) or v < 0 for v in tolerance):
                    raise ValueError("fit_tolerance requires one or one-per-joint finite nonnegative value")
                targets = TimedJointTargets(
                    positions=tuple(q for _, q, _ in points), origin_ns=message.target_origin_ns,
                    period_ns=message.sample_period_ns, basis=prediction_basis,
                    lead_ns=message.lead_ns, join_seconds=message.join_ns / 1e9,
                    fit_tolerance=tolerance[0] if len(tolerance) == 1 else tolerance,
                    predecessor=predecessor, joints=tuple(self.description["joints"]),
                )
                operation = self.session.timed_joint_targets(targets, **kwargs)
                self._event("motion.prepared", {"source": source.name, "command_id": message.command_id,
                                               "fit": operation.fit_report, "prediction_basis": message.prediction_observation_sequence}, basis)
            else:
                raise ValueError("unknown motion kind")
            self.pending.append(Pending(operation, source, 0, message.command_id, "motion", self.session.generation, key))
            self.input_blocked = False
            self._reply(source, 0, message.command_id, "motion", "queued", generation=self.session.generation,
                        native_id=operation.command_id, key=key)
        except (ValueError, TypeError, RuntimeError) as error:
            self._reply(source, 0, message.command_id, "motion", "rejected", str(error), message.session_generation, key=key)

    def _handle_control(self, value):
        request_id, action, name, instance = value
        source = None
        operation_name = "control"
        try:
            operations = {Control.Request.ENABLE: "enable", Control.Request.DISABLE: "disable", Control.Request.STOP: "stop",
                          Control.Request.HOLD: "hold", Control.Request.STANDBY: "standby", Control.Request.OPEN_GRIPPER: "open_gripper",
                          Control.Request.CLOSE_GRIPPER: "close_gripper"}
            if action in operations:
                operation_name = operations[action]
                operation = getattr(self.robot, operation_name)()
                if action in (Control.Request.DISABLE, Control.Request.STOP, Control.Request.HOLD, Control.Request.STANDBY):
                    self.selected = None
                    self.session = None
                    self.input_blocked = None
            elif action in (Control.Request.SELECT, Control.Request.TAKEOVER):
                operation_name = "select" if action == Control.Request.SELECT else "takeover"
                source = self._source(name, instance)
                if time.monotonic() - source.seen >= 2.0:
                    raise ValueError("source heartbeat is no longer live")
                if action == Control.Request.SELECT and self.session is not None and self.session.valid:
                    raise ValueError("a source is already selected; use release or explicit takeover")
                operation = source.handle.acquire_control(takeover=action == Control.Request.TAKEOVER)
            elif action == Control.Request.RELEASE:
                operation_name = "release"
                source = self.selected
                if source is None or self.session is None:
                    raise ValueError("no selected source to release")
                if name and (name, instance) != (source.name, source.instance):
                    raise ValueError("release must name the selected source")
                operation = source.handle.release(self.session)
                self.selected = None
                self.session = None
                self.input_blocked = None
            elif action == Control.Request.STATUS:
                self._publish_state(self.robot.observe())
                self._reply(None, request_id, 0, "status", "completed")
                return
            else:
                raise ValueError("unknown control action")
            if operation is None:
                self._reply(source, request_id, 0, operation_name, "completed")
                return
            self.pending.append(Pending(operation, source, request_id, 0, operation_name, 0))
            self._reply(source, request_id, 0, operation_name, "queued")
        except (ValueError, TypeError, RuntimeError) as error:
            self._reply(source, request_id, 0, operation_name, "rejected", str(error))

    def _poll_operations(self):
        remaining = []
        for pending in self.pending:
            try:
                result = pending.operation.outcome
                outcome = result.outcome.value
                if outcome == "completed" and pending.name in ("select", "takeover"):
                    session = pending.operation.result(timeout=0)
                    self.selected, self.session = pending.source, session
                    self.input_blocked = None
                    pending.session_generation = session.generation
                    self._event("authority.selected", self._source_metadata(self.selected))
                if outcome != pending.last_outcome:
                    self._reply(pending.source, pending.request_id, pending.command_id, pending.name, outcome, result.reason,
                                pending.session_generation, getattr(pending.operation, "command_id", 0), pending.key)
                    pending.last_outcome = outcome
                if pending.name == "motion" and outcome in ("accepted", "completed"):
                    key = (pending.source.name, pending.source.instance, pending.session_generation, pending.command_id)
                    self.predecessors[key] = pending.operation
                    while len(self.predecessors) > 256:
                        self.predecessors.popitem(last=False)
                if not pending.operation.done:
                    remaining.append(pending)
            except (ValueError, TypeError, RuntimeError) as error:
                self._reply(pending.source, pending.request_id, pending.command_id, pending.name, "rejected", str(error),
                            pending.session_generation, key=pending.key)
        self.pending = remaining
        if self.session is not None and not self.session.valid:
            self._event("authority.lost", self._source_metadata(self.selected))
            self.selected = None
            self.session = None
            self.input_blocked = None

    def _identity_result(self, value):
        return {"success": True, "error": "", "runtime_instance": value.identity.runtime_instance,
                "timeline_generation": value.epoch, "observation_sequence": value.sequence,
                "execution_time_ns": int(value.to_record()["time_ns"])}

    def _owner_job(self, name, arguments, future):
        try:
            if name == "step":
                if self.mode != "stepped":
                    raise ValueError("step requires stepped simulation")
                if not 1 <= arguments <= 1000:
                    raise ValueError("step n must be between one and 1000")
                for _ in range(arguments):
                    if self.closed.is_set():
                        raise ValueError("server is closing")
                    self._drain_heartbeats()
                    self._drain_urgent()
                    value = self.robot.step()
                    self._poll_operations()
                    self._publish_state(value)
                    self._publish_capture(value)
            elif name == "reset":
                if self.backend != "mujoco":
                    raise ValueError("hardware has no simulation reset capability")
                value = self.robot.reset()
                self.selected, self.session = None, None
                self.input_blocked = None
                self.predecessors.clear()
                self.bases.clear()
                self.last_capture = None
                self._poll_operations()
                self._publish_state(value)
                self._publish_static(value)
                self._event("runtime.reset", basis=value)
            else:
                value = self.robot.capture() if self.configuration.cameras or self.configuration.external_sensors else self.robot.observe()
                self._publish_state(value)
                self._publish_capture(value, repeat=True)
            future.set_result(self._identity_result(value))
        except Exception as error:
            future.set_result({"success": False, "error": str(error)})

    def _dispatch(self, kind, value):
        if kind == "motion":
            self._handle_motion(*value)
        elif kind == "heartbeat":
            message, received = value
            try:
                self._handle_heartbeat(message, received)
            except (ValueError, TypeError, RuntimeError) as error:
                self._event("input.rejected", {"source": message.source, "reason": str(error)})
        elif kind == "control":
            self._handle_control(value)
        else:
            arguments, future = value
            self._owner_job(kind, arguments, future)

    def _drain_urgent(self):
        while True:
            try:
                kind, value = self.urgent.get_nowait()
            except Empty:
                return
            self._dispatch(kind, value)

    def _drain_heartbeats(self):
        for _ in range(128):
            try:
                kind, value = self.heartbeat_jobs.get_nowait()
            except Empty:
                return
            self._dispatch(kind, value)

    def _work(self):
        next_publish = time.monotonic()
        previous_error = None
        try:
            while not self.closed.is_set():
                try:
                    self._drain_heartbeats()
                    self._drain_urgent()
                    try:
                        kind, value = self.jobs.get(timeout=0.01)
                        self._dispatch(kind, value)
                    except Empty:
                        pass
                    self._poll_operations()
                    if time.monotonic() >= next_publish:
                        self._publish_state(self.robot.observe())
                        if self.mode == "realtime" and (self.configuration.cameras or self.configuration.external_sensors):
                            try:
                                self._publish_capture(self.robot.capture(timeout=0.05))
                                self.camera_error = None
                            except RuntimeError as error:
                                self.camera_error = str(error)
                        next_publish = time.monotonic() + self.publish_period
                    self.server_error = None
                except Exception as error:
                    self.server_error = str(error)
                    if self.server_error != previous_error:
                        self.get_logger().error(self.server_error)
                        previous_error = self.server_error
        finally:
            for queue in (self.jobs, self.heartbeat_jobs, self.urgent):
                while True:
                    try:
                        kind, value = queue.get_nowait()
                    except Empty:
                        break
                    if kind in ("step", "reset", "capture"):
                        value[1].set_result({"success": False, "error": "server is closing"})
            self.robot.close()

    def destroy_node(self):
        with self.lock:
            self.closed.set()
        self.worker.join()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = RobotServer()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            with node.lock:
                node.closed.set()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
