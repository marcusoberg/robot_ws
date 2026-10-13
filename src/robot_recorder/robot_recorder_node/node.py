"""Recorder owns episodes; Robot Server remains independent of recording state."""

from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from threading import RLock
from uuid import uuid4
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from robot_server.msg import Capture, Observation, Event as ServerEvent, CommandResult
from robot_recorder.srv import Control, Acknowledge
from recording_tools.recording import Recorder
from recording_tools.contracts import (
    Identity,
    Episode,
    Event,
    CaptureRecord,
    RecordingStatus,
)
from recording_tools.contracts.transport import decode_server_capture
from .qos import EVENTS, LATEST, DESCRIPTION, CAPTURES
from .schema import definition
from sensor_msgs.msg import JointState, Image, CameraInfo, CompressedImage
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Float64MultiArray


class RecordingNode(Node):
    def __init__(self):
        super().__init__("robot_recorder")
        self.declare_parameter("capacity", 64)
        self.declare_parameter("output_root", "runs")
        self.lock = RLock()
        self.worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="recording-finalizer"
        )
        self.recorder = None
        self.episode = None
        self.first_capture = None
        self.last_episode_capture = None
        self.finish_request = None
        self.stop_requested = None
        self.basis = None
        self.pending = {}
        self.high_water = {}
        self.capture_episodes = {}
        self.event_tickets = {}
        self.event_documents = {}
        self.finalized = {}
        self.operations = {}
        self.stop_operation = None
        self.last_manifest = None
        self.frames = self.gaps = 0
        self.last = None
        self.host_clock = "recorder.host.monotonic:" + uuid4().hex
        self.capture_group = MutuallyExclusiveCallbackGroup()
        self.status_publisher = self.create_publisher(
            String, "/recording/status", LATEST
        )
        self.episode_publisher = self.create_publisher(
            String, "/recording/episode", DESCRIPTION
        )
        self.event_publisher = self.create_publisher(
            String, "/recording/events", EVENTS
        )
        self.create_timer(0.5, self.publish_status)
        self.create_timer(0.05, self.advance_operations)
        self.create_subscription(
            Capture,
            "/robot/capture",
            self.capture,
            CAPTURES,
            callback_group=self.capture_group,
        )
        self.create_subscription(
            Observation,
            "/robot/state",
            self.observation,
            LATEST,
            callback_group=self.capture_group,
        )
        self.create_subscription(
            ServerEvent,
            "/robot/events",
            self.server_event,
            EVENTS,
            callback_group=self.capture_group,
        )
        self.create_service(
            Control,
            "/recording/control",
            self.control,
            callback_group=self.capture_group,
        )
        self.create_service(
            Acknowledge, "/recording/ack", self.ack, callback_group=self.capture_group
        )
        self.projected = set()
        for topic, kind, qos in (
            ("/joint_states", JointState, LATEST),
            ("/robot/reference", JointState, LATEST),
            ("/robot/tracking_error", Float64MultiArray, LATEST),
            ("/tf", TFMessage, LATEST),
            ("/tf_static", TFMessage, DESCRIPTION),
            ("/robot/urdf", String, DESCRIPTION),
            ("/robot/description", String, DESCRIPTION),
            ("/robot/ownership", String, LATEST),
            ("/robot/timing", Float64MultiArray, LATEST),
            ("/robot/state", Observation, LATEST),
            ("/robot/events", ServerEvent, EVENTS),
            ("/robot/result", CommandResult, EVENTS),
        ):
            self.project(topic, kind, qos)
        self.create_timer(1.0, self.discover)
        self.publish_episode()

    def publish_episode(self):
        self.episode_publisher.publish(
            String(data=self.episode.model_dump_json() if self.episode else "null")
        )

    def publish_status(self):
        with self.lock:
            recorder = self.recorder
            value = RecordingStatus(
                host_ns=time.monotonic_ns(),
                active=recorder is not None,
                frames=self.frames,
                capture_gaps=self.gaps,
                dropped=recorder.manifest["dropped"] if recorder else 0,
                queue_depth=recorder.queue.qsize() if recorder else 0,
                error=str(recorder.error) if recorder and recorder.error else None,
            )
            self.status_publisher.publish(String(data=value.model_dump_json()))

    def project(self, topic, kind, qos):
        name = kind.__module__.split(".")[0] + "/msg/" + kind.__name__
        schema = definition(name)

        def record(serialized):
            with self.lock:
                if self.recorder:
                    self.recorder.append(
                        topic,
                        serialized,
                        encoding="cdr",
                        schema=schema,
                        schema_name=name,
                        schema_encoding="ros2msg",
                    )

        self.create_subscription(kind, topic, record, qos, raw=True)
        self.projected.add(topic)

    def discover(self):
        kinds = {
            "sensor_msgs/msg/Image": Image,
            "sensor_msgs/msg/CompressedImage": CompressedImage,
            "sensor_msgs/msg/CameraInfo": CameraInfo,
        }
        for topic, types in self.get_topic_names_and_types():
            if (
                topic.startswith("/robot/cameras/")
                and topic not in self.projected
                and types
                and types[0] in kinds
            ):
                self.project(topic, kinds[types[0]], LATEST)

    def _boundary(self, identity):
        stream = (identity.runtime_instance, identity.timeline_generation)
        if (
            self.basis
            and stream != (self.basis.runtime_instance, self.basis.timeline_generation)
            and self.episode
        ):
            first_stream = (
                (
                    self.first_capture.runtime_instance,
                    self.first_capture.timeline_generation,
                )
                if self.first_capture
                else None
            )
            if stream != first_stream:
                self._abort_episode("runtime_or_timeline_changed")
        self.basis = identity

    def _episode_for(self, identity):
        if self.episode is None:
            return None
        stream = (identity.runtime_instance, identity.timeline_generation)
        if self.first_capture:
            if (
                stream
                != (
                    self.first_capture.runtime_instance,
                    self.first_capture.timeline_generation,
                )
                or identity.observation_sequence
                < self.first_capture.observation_sequence
            ):
                return None
        request = self.finish_request
        if request and request["phase"] == "capture":
            cutoff = request["cutoff"]
            if (
                stream != (cutoff.runtime_instance, cutoff.timeline_generation)
                or identity.observation_sequence > cutoff.observation_sequence
            ):
                return None
        self.last_episode_capture = identity
        return self.episode

    def observation(self, message):
        with self.lock:
            try:
                record = json.loads(message.record_json)
                identity = Identity.model_validate(record["identity"])
                expected = Identity(
                    runtime_instance=message.runtime_instance,
                    timeline_generation=message.timeline_generation,
                    observation_sequence=message.observation_sequence,
                )
                if (
                    identity != expected
                    or record["time_ns"] != message.execution_time_ns
                ):
                    raise ValueError(
                        "Observation envelope disagrees with canonical basis"
                    )
                if self.recorder is None:
                    self._boundary(identity)
            except (ValueError, TypeError, KeyError) as error:
                self.get_logger().warning(f"Rejected observation: {error}")

    def capture(self, message):
        with self.lock:
            recorder = self.recorder
            if recorder is None:
                return
            key = (
                message.runtime_instance,
                message.timeline_generation,
                message.observation_sequence,
            )
            # CaptureOnce may resend exactly the same immutable frame. Reuse its ticket.
            if key in self.pending:
                return
            stream = key[:2]
            if key[2] <= self.high_water.get(stream, -1):
                return
            received = time.monotonic_ns()
            try:
                identity, observation, images = decode_server_capture(
                    bytes(message.packet)
                )
                expected = Identity(
                    runtime_instance=key[0],
                    timeline_generation=key[1],
                    observation_sequence=key[2],
                )
                if (
                    identity != expected
                    or observation["time_ns"] != message.execution_time_ns
                ):
                    raise ValueError(
                        "Capture envelope disagrees with original observation"
                    )
                if not message.host_clock or message.host_published_ns < 0:
                    raise ValueError("Capture publisher clock is unavailable")
                self._boundary(identity)
                episode = self._episode_for(identity)
                self.capture_episodes[key] = episode.id if episode else None
                record = CaptureRecord(
                    identity=identity,
                    episode=episode,
                    observation=observation,
                    stream_sequence=message.delivery_sequence,
                    host_received_ns=received,
                    host_published_ns=time.monotonic_ns(),
                    host_clock=self.host_clock,
                    publisher_host_ns=message.host_published_ns,
                    publisher_host_clock=message.host_clock,
                )
                current = (stream, message.delivery_sequence)
                if self.last and current[0] == self.last[0]:
                    self.gaps += max(0, current[1] - self.last[1] - 1)
                self.last = current
                ticket = recorder.capture(record, images)
                self.frames += 1
            except Exception as error:
                ticket = Future()
                ticket.set_exception(ValueError(f"Malformed capture: {error}"))
                recorder.manifest["dropped"] += 1
                self.get_logger().warning(f"Rejected capture: {error}")
            self.high_water[stream] = key[2]
            self.pending[key] = ticket
            if len(self.pending) > 4096:
                expired = next(iter(self.pending))
                self.pending.pop(expired)
                self.capture_episodes.pop(expired, None)
            self._advance_operations()

    def _record_event(self, event):
        document = event.model_dump_json()
        if event.event_id in self.event_documents:
            if self.event_documents[event.event_id] != document:
                raise ValueError("An event id was reused for different content")
            return self.event_tickets[event.event_id]
        if self.recorder is None:
            raise ValueError("No active recording")
        ticket = self.recorder.event(event)
        self.event_documents[event.event_id] = document
        self.event_tickets[event.event_id] = ticket
        self.event_publisher.publish(String(data=document))
        return ticket

    def _event(self, kind, *, episode=None, details=None, event_id=None, basis=None):
        event = Event(
            event_id=event_id or uuid4().hex,
            kind=kind,
            host_ns=time.monotonic_ns(),
            host_clock=self.host_clock,
            basis=basis if basis is not None else self.basis,
            episode=episode,
            details=details or {},
        )
        self._record_event(event)
        return event.event_id

    def server_event(self, message):
        with self.lock:
            if self.recorder is None:
                return
            try:
                basis = (
                    Identity(
                        runtime_instance=message.runtime_instance,
                        timeline_generation=message.timeline_generation,
                        observation_sequence=message.observation_sequence,
                    )
                    if message.has_basis
                    else None
                )
                original = (
                    Event.model_validate_json(self.event_documents[message.event_id])
                    if message.event_id in self.event_documents
                    else None
                )
                event = Event(
                    event_id=message.event_id,
                    kind=message.kind,
                    host_ns=message.host_ns,
                    host_clock=message.host_clock,
                    basis=basis,
                    episode=original.episode if original else self.episode,
                    details=json.loads(message.details_json),
                )
                self._record_event(event)
            except (ValueError, TypeError, KeyError) as error:
                self.recorder.manifest["dropped"] += 1
                self.get_logger().warning(f"Rejected Robot Server event: {error}")

    @staticmethod
    def _ticket_result(ticket):
        if not ticket.done():
            return {"pending": True, "recorded": False}
        try:
            result = ticket.result()
            return {"pending": False, "recorded": True, "result": result}
        except Exception as error:
            return {"pending": False, "recorded": False, "error": str(error)}

    def ack(self, request, response):
        with self.lock:
            key = (
                request.runtime_instance,
                request.timeline_generation,
                request.observation_sequence,
            )
            ticket = self.pending.get(key)
            if ticket is None:
                if self.recorder is None:
                    response.error = (
                        "No active recording or retained capture acknowledgement"
                    )
                elif key[2] <= self.high_water.get(key[:2], -1):
                    response.error = "Capture acknowledgement expired"
                else:
                    response.pending = True
                return response
            result = self._ticket_result(ticket)
            response.pending, response.recorded = result["pending"], result["recorded"]
            response.error = result.get("error", "")
            return response

    def _abort_episode(self, reason):
        request = self.finish_request
        if self.episode:
            final = Episode.model_validate(
                {**self.episode.model_dump(), "approved": False, "success": None}
            )
            event_id = self._event(
                "episode.finish",
                episode=final,
                details={"reason": reason},
                event_id=request["event_id"] if request else None,
            )
            self.finalized[final.id] = {
                "episode": final.model_dump(),
                "event_id": event_id,
            }
        if request and not request["future"].done():
            request["future"].set_exception(RuntimeError(reason))
        self.episode = self.first_capture = self.last_episode_capture = (
            self.finish_request
        ) = None
        self.publish_episode()

    def _finish(self, args, *, reason="operator"):
        expected = args.get("id")
        if self.finish_request:
            request = self.finish_request
            if expected is not None and expected != request["episode"].id:
                raise ValueError("A different episode is finalizing")
            if args.get("event_id") not in (None, request["event_id"]):
                raise ValueError("Finalization event id changed")
            if (
                "final_capture" in args
                and Identity.model_validate(args["final_capture"]) != request["cutoff"]
            ):
                raise ValueError("Final capture changed during finalization")
            for name in ("approved", "success"):
                if name in args and args[name] != getattr(request["episode"], name):
                    raise ValueError("Episode outcome changed during finalization")
            return {
                "ticket": request["ticket"],
                "event_id": request["event_id"],
                **self._ticket_result(request["future"]),
            }
        if self.episode is None:
            if expected in self.finalized:
                result = self.finalized[expected]
                return {
                    **result,
                    **self._ticket_result(self.event_tickets[result["event_id"]]),
                }
            raise ValueError("No active episode")
        if expected is not None and expected != self.episode.id:
            raise ValueError("Episode context changed")
        cutoff = (
            Identity.model_validate(args["final_capture"])
            if "final_capture" in args
            else self.last_episode_capture
        )
        if cutoff is None:
            raise ValueError(
                "Declare final_capture or capture at least one episode frame before finishing"
            )
        if self.first_capture:
            first = self.first_capture
            if (cutoff.runtime_instance, cutoff.timeline_generation) != (
                first.runtime_instance,
                first.timeline_generation,
            ) or cutoff.observation_sequence < first.observation_sequence:
                raise ValueError("Final capture precedes the declared first capture")
        key = (
            cutoff.runtime_instance,
            cutoff.timeline_generation,
            cutoff.observation_sequence,
        )
        if key in self.pending and self.capture_episodes.get(key) != self.episode.id:
            raise ValueError("Final capture was already recorded outside this episode")
        if self.last_episode_capture:
            latest = self.last_episode_capture
            if (cutoff.runtime_instance, cutoff.timeline_generation) != (
                latest.runtime_instance,
                latest.timeline_generation,
            ) or cutoff.observation_sequence < latest.observation_sequence:
                raise ValueError(
                    "Final capture precedes frames already associated with this episode"
                )
        changes = {k: args[k] for k in ("approved", "success") if k in args}
        final = Episode.model_validate({**self.episode.model_dump(), **changes})
        event_id = args.get("event_id") or uuid4().hex
        if not isinstance(event_id, str) or not event_id:
            raise ValueError("Event id must be a nonempty string")
        if event_id in self.event_documents:
            raise ValueError("Finalization event id was already used")
        future, ticket = Future(), uuid4().hex
        self.finish_request = {
            "episode": final,
            "cutoff": cutoff,
            "future": future,
            "event_id": event_id,
            "ticket": ticket,
            "phase": "capture",
            "reason": reason,
        }
        self.operations[ticket] = future
        self.event_tickets[event_id] = future
        self._advance_operations()
        return {"ticket": ticket, "event_id": event_id, **self._ticket_result(future)}

    def _start_stop(self):
        if not self.stop_requested or self.finish_request or self.recorder is None:
            return
        recorder, self.recorder = self.recorder, None
        self._record_stop_event(recorder)
        recorder.manifest["capture_gaps"], recorder.manifest["frames"] = (
            self.gaps,
            self.frames,
        )
        target = self.operations[self.stop_requested]

        def finalize():
            try:
                recorder.close()
            except RuntimeError:
                pass
            with self.lock:
                self.last_manifest = recorder.manifest
            return recorder.manifest

        work = self.worker.submit(finalize)

        def finished(value):
            try:
                target.set_result(value.result())
            except Exception as error:
                target.set_exception(error)

        work.add_done_callback(finished)
        self.stop_requested = None

    def _record_stop_event(self, recorder):
        event = Event(
            kind="record.stop",
            host_ns=time.monotonic_ns(),
            host_clock=self.host_clock,
            basis=self.basis,
        )
        recorder.event(event)
        self.event_publisher.publish(String(data=event.model_dump_json()))

    def _advance_operations(self):
        request = self.finish_request
        if request:
            if request["phase"] == "capture":
                cutoff = request["cutoff"]
                key = (
                    cutoff.runtime_instance,
                    cutoff.timeline_generation,
                    cutoff.observation_sequence,
                )
                capture = self.pending.get(key)
                if capture is None and self.high_water.get(key[:2], -1) > key[2]:
                    self._abort_episode("declared_final_capture_missing")
                elif capture is not None and capture.done():
                    try:
                        capture.result()
                    except Exception:
                        self._abort_episode("declared_final_capture_not_recorded")
                    else:
                        event_id = self._event(
                            "episode.finish",
                            episode=request["episode"],
                            details={
                                "reason": request["reason"],
                                "final_capture": cutoff.model_dump(),
                            },
                            event_id=request["event_id"],
                            basis=cutoff,
                        )
                        request["event_ticket"] = self.event_tickets[event_id]
                        request["phase"] = "event"
                        self.episode = self.first_capture = (
                            self.last_episode_capture
                        ) = None
                        self.publish_episode()
            if (
                self.finish_request
                and request["phase"] == "event"
                and request["event_ticket"].done()
            ):
                try:
                    request["event_ticket"].result()
                    result = {
                        "episode": request["episode"].model_dump(),
                        "event_id": request["event_id"],
                        "final_capture": request["cutoff"].model_dump(),
                    }
                    self.finalized[request["episode"].id] = result
                    request["future"].set_result(result)
                except Exception as error:
                    request["future"].set_exception(error)
                self.finish_request = None
        self._start_stop()

    def advance_operations(self):
        with self.lock:
            self._advance_operations()

    def _control(self, operation, args):
        if operation == "status":
            closing = bool(
                self.stop_operation and not self.operations[self.stop_operation].done()
            )
            return {
                "active": self.recorder is not None,
                "closing": closing,
                "frames": self.frames,
                "gaps": self.gaps,
                "drops": self.recorder.manifest["dropped"]
                if self.recorder
                else (self.last_manifest or {}).get("dropped", 0),
                "episode": self.episode.model_dump() if self.episode else None,
                "basis": self.basis.model_dump() if self.basis else None,
                "stop_ticket": self.stop_operation,
            }
        if operation in {"operation.ack", "event.ack"}:
            tickets = (
                self.operations if operation == "operation.ack" else self.event_tickets
            )
            key = (
                args.get("ticket")
                if operation == "operation.ack"
                else args.get("event_id")
            )
            if key not in tickets:
                raise ValueError("Unknown acknowledgement ticket")
            return self._ticket_result(tickets[key])
        if operation == "record.start":
            if self.recorder or (
                self.stop_operation and not self.operations[self.stop_operation].done()
            ):
                raise ValueError("Recording is active or finalizing")
            directory = Path(
                args.get(
                    "directory",
                    Path(self.get_parameter("output_root").value)
                    / ("recording-" + time.strftime("%Y%m%d-%H%M%S")),
                )
            ).resolve()
            capacity = args.get("capacity", self.get_parameter("capacity").value)
            if type(capacity) is not int or capacity <= 0:
                raise ValueError("Recording capacity must be positive")
            manifest = args.get("manifest", {})
            if not isinstance(manifest, dict):
                raise ValueError("Recording manifest must be a mapping")
            self.recorder = Recorder(
                directory,
                {**manifest, "recorder_host_clock": self.host_clock},
                capacity,
            )
            self.frames = self.gaps = 0
            self.last = None
            self.pending, self.high_water, self.capture_episodes = {}, {}, {}
            self.event_tickets, self.event_documents, self.finalized = {}, {}, {}
            self.episode = self.first_capture = self.last_episode_capture = (
                self.finish_request
            ) = None
            self.stop_operation = self.stop_requested = None
            self._event("record.start")
            self.publish_episode()
            return {"directory": str(directory)}
        if operation == "record.stop":
            if self.stop_operation:
                return {
                    "ticket": self.stop_operation,
                    **self._ticket_result(self.operations[self.stop_operation]),
                }
            if self.recorder is None:
                raise ValueError("No active recording")
            if self.episode and not self.finish_request:
                if self.last_episode_capture:
                    self._finish(
                        {"approved": False, "success": None}, reason="recording_stopped"
                    )
                else:
                    self._abort_episode("recording_stopped_before_capture")
            ticket = uuid4().hex
            self.operations[ticket] = Future()
            self.stop_operation = self.stop_requested = ticket
            self._advance_operations()
            return {"ticket": ticket, **self._ticket_result(self.operations[ticket])}
        if self.recorder is None:
            raise ValueError("Episode operations require an active recording")
        if operation == "episode.start":
            if self.finish_request or self.stop_requested:
                raise ValueError("Recording or episode is finalizing")
            first = (
                Identity.model_validate(args["first_capture"])
                if "first_capture" in args
                else None
            )
            values = {
                k: v for k, v in args.items() if k not in {"event_id", "first_capture"}
            }
            values.setdefault("id", uuid4().hex)
            episode = Episode.model_validate(values)
            if self.episode and self.episode.id == episode.id:
                if self.episode != episode or self.first_capture != first:
                    raise ValueError(
                        "Episode id reused for different metadata or first capture"
                    )
                return {"episode": episode.model_dump(), "unchanged": True}
            if episode.id in self.finalized:
                raise ValueError("Episode id was already finalized")
            if first and first.observation_sequence <= self.high_water.get(
                (first.runtime_instance, first.timeline_generation), -1
            ):
                raise ValueError(
                    "First capture was already recorded; declare a future identity"
                )
            event_id = args.get("event_id") or uuid4().hex
            if (
                not isinstance(event_id, str)
                or not event_id
                or event_id in self.event_documents
            ):
                raise ValueError("Episode start requires an unused nonempty event id")
            if self.episode:
                self._event(
                    "episode.suspended",
                    episode=self.episode,
                    details={"next_episode": episode.id},
                )
            self.episode, self.first_capture, self.last_episode_capture = (
                episode,
                first,
                None,
            )
            event_id = self._event(
                "episode.start",
                episode=episode,
                event_id=event_id,
                details={"first_capture": first.model_dump() if first else None},
            )
            self.publish_episode()
            return {
                "episode": episode.model_dump(),
                "event_id": event_id,
                **self._ticket_result(self.event_tickets[event_id]),
            }
        if operation == "episode.finish":
            return self._finish(args)
        if operation == "episode.suspend":
            if self.finish_request:
                raise ValueError("Episode is finalizing")
            if self.episode is None:
                raise ValueError("No active episode")
            event_id = self._event(
                "episode.suspended", episode=self.episode, details=args
            )
            self.episode = self.first_capture = self.last_episode_capture = None
            self.publish_episode()
            return {
                "event_id": event_id,
                **self._ticket_result(self.event_tickets[event_id]),
            }
        if operation == "event":
            event = Event.model_validate(args)
            ticket = self._record_event(event)
            return {"event_id": event.event_id, **self._ticket_result(ticket)}
        raise ValueError("Unsupported recording operation")

    def control(self, request, response):
        with self.lock:
            try:
                if len(request.arguments_json) > 1024 * 1024:
                    raise ValueError("Recording request exceeds metadata budget")
                args = json.loads(request.arguments_json or "{}")
                if not isinstance(args, dict):
                    raise ValueError("Recording arguments must be a mapping")
                result = self._control(request.operation, args)
                response.applied = True
                response.result_json = json.dumps(result)
            except Exception as error:
                response.error = str(error)
            return response

    def destroy_node(self):
        with self.lock:
            if self.finish_request:
                self._advance_operations()
                if self.finish_request:
                    self._abort_episode("recorder_shutdown_before_finalization_ack")
            if self.recorder:
                if self.episode:
                    self._abort_episode("recorder_shutdown")
                if self.stop_operation is None:
                    self.stop_operation = uuid4().hex
                    self.operations[self.stop_operation] = Future()
                self.stop_requested = self.stop_operation
                self._start_stop()
        self.worker.shutdown(wait=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = RecordingNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    print("Recorder ready", flush=True)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
