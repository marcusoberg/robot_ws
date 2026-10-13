"""Small ROS-only producer facade. It never enables or selects itself."""

import math
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_server.msg import Observation, Heartbeat, MotionCommand, CommandResult
from std_msgs.msg import String

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
DESCRIPTION = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Producer(Node):
    def __init__(self, source):
        super().__init__(source.replace("-", "_") + "_source")
        self.source, self.instance = source, str(uuid.uuid4())
        self.state, self.description, self.sequence = None, None, 0
        self.results = {}
        self.create_subscription(Observation, "/robot/state", self._state, LATEST)
        self.create_subscription(
            String, "/robot/description", self._description, DESCRIPTION
        )
        self.create_subscription(CommandResult, "/robot/result", self._result, 32)
        self.commands = self.create_publisher(MotionCommand, "/robot/command", 1)
        self.heartbeats = self.create_publisher(Heartbeat, "/robot/heartbeat", 1)
        self.create_timer(0.1, self.heartbeat)

    def _state(self, message):
        try:
            value = json.loads(message.record_json)
            if not isinstance(value, dict):
                raise ValueError("Observation must be a mapping")
            identity = value.get("identity")
            if not isinstance(identity, dict) or (
                identity.get("runtime_instance"),
                identity.get("timeline_generation"),
                identity.get("observation_sequence"),
                value.get("time_ns"),
            ) != (
                message.runtime_instance,
                message.timeline_generation,
                message.observation_sequence,
                message.execution_time_ns,
            ):
                raise ValueError("Observation JSON and ROS header disagree")
            if (
                any(
                    type(identity.get(key)) is not int or identity[key] < 0
                    for key in ("timeline_generation", "observation_sequence")
                )
                or type(value.get("time_ns")) is not int
            ):
                raise ValueError("Observation identity/time must use integer values")
            selected = value.get("selected_source")
            if selected is not None and (
                not isinstance(selected, dict)
                or any(
                    not isinstance(selected.get(key), str) or not selected[key]
                    for key in ("source", "source_instance")
                )
                or type(selected.get("session_generation")) is not int
                or selected["session_generation"] <= 0
            ):
                raise ValueError("Invalid selected producer")
            request = value.get("actuator_request")
            if (
                not isinstance(request, dict)
                or not isinstance(request.get("position"), list)
                or any(
                    type(q) not in (int, float) or not math.isfinite(q)
                    for q in request["position"]
                )
            ):
                raise ValueError("Invalid actuator reference")
            schedule = value.get("schedule")
            if schedule is not None and not isinstance(schedule, dict):
                raise ValueError("Invalid schedule metadata")
            self.state = value
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().error("Observation rejected: " + str(error))

    def _description(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("Description must be a mapping")
            joints = value.get("joints")
            if (
                not isinstance(joints, list)
                or not joints
                or any(not isinstance(j, str) or not j for j in joints)
                or len(set(joints)) != len(joints)
            ):
                raise ValueError("Invalid joint names")
            for key in ("lower", "upper"):
                coordinates = value.get(key)
                if (
                    not isinstance(coordinates, list)
                    or len(coordinates) != len(joints)
                    or any(
                        type(q) not in (int, float) or not math.isfinite(q)
                        for q in coordinates
                    )
                ):
                    raise ValueError("Invalid joint limits")
            if any(
                lower > upper for lower, upper in zip(value["lower"], value["upper"])
            ):
                raise ValueError("Reversed joint limits")
            self.description = value
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().error("Description rejected: " + str(error))

    def _result(self, message):
        labels = ("queued", "accepted", "completed", "rejected", "canceled")
        if not 0 <= message.outcome < len(labels):
            self.get_logger().error("Result has an unknown outcome")
            return
        if message.source == self.source and message.source_instance == self.instance:
            self.results[message.command_id] = message
            if len(self.results) > 128:
                self.results.pop(next(iter(self.results)))
            if message.outcome != CommandResult.QUEUED:
                self.get_logger().info(
                    f"command {message.command_id}: {labels[message.outcome]} {message.reason}"
                )

    @property
    def selected(self):
        source = (self.state or {}).get("selected_source")
        return (
            source
            if source
            and source["source"] == self.source
            and source["source_instance"] == self.instance
            else None
        )

    def heartbeat(self):
        if not self.state:
            return
        identity = self.state["identity"]
        self.sequence += 1
        self.heartbeats.publish(
            Heartbeat(
                source=self.source,
                source_instance=self.instance,
                runtime_instance=identity["runtime_instance"],
                timeline_generation=int(identity["timeline_generation"]),
                session_generation=(self.selected or {}).get("session_generation", 0),
                sequence=self.sequence,
                basis_observation_sequence=int(identity["observation_sequence"]),
                ttl_ns=300_000_000,
            )
        )

    def wait_ready(self, timeout=10):
        until = time.monotonic() + timeout
        while rclpy.ok() and (self.state is None or self.description is None):
            if time.monotonic() >= until:
                raise TimeoutError(
                    "Robot state/description unavailable. Launch robot_server first."
                )
            rclpy.spin_once(self, timeout_sec=0.1)

    def command(self, kind, *, deadman=True, clutch=False):
        if not self.selected:
            return None
        identity = self.state["identity"]
        self.sequence += 1
        return MotionCommand(
            source=self.source,
            source_instance=self.instance,
            runtime_instance=identity["runtime_instance"],
            timeline_generation=int(identity["timeline_generation"]),
            session_generation=self.selected["session_generation"],
            sequence=self.sequence,
            command_id=self.sequence,
            basis_observation_sequence=int(identity["observation_sequence"]),
            ttl_ns=300_000_000,
            deadman=deadman,
            clutch=clutch,
            kind=kind,
            joint_names=self.description["joints"],
        )

    def positions(self):
        return list(self.state["actuator_request"]["position"])
