"""ROS execution server. Backends own IO and stopping; producers own goals."""

from collections import deque
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String

from robot_description.spec import load_profile
from robot_server.msg import Status, JointGoal, Control, TrackingError
from robot_server_core.application import Application
from robot_server_core.state import CommandResult
from robot_server_core.observation import ObservationWorker
from robot_server_core.serial_backend import SerialApplication
from robot_server_node.visualization import Visualization
from robot_server_node.ownership import acquire_server_lock, describe_owner



class Adapter(Node):
    def __init__(self, app=None):
        super().__init__("robot_server", start_parameter_services=False)
        self.server_lock = acquire_server_lock() if app is None else None
        robot = self.declare_parameter("robot", "so101").value
        gripper = self.declare_parameter("gripper", "pneumatic").value
        scene = self.declare_parameter("scene", "pick_place").value
        backend = self.declare_parameter("backend", "simulation").value
        cameras = self.declare_parameter("cameras", False).value
        recordings = self.declare_parameter("recordings", "recordings").value
        port = self.declare_parameter("serial_port", "").value
        tool_isolated = self.declare_parameter("tool_isolated", False).value
        self.paced = self.declare_parameter("paced", False).value
        self.owns_application = app is None
        self.serial = backend == "serial" and self.owns_application
        self.visualizer = None
        self.worker = None
        self.last_backend_mode = None
        self.last_estop_state = None
        self.command_id = ""
        self.command_result = ""
        self.command_accepted = False
        self.seen_commands = deque(maxlen=128)

        if app is not None:
            if self.paced:
                raise ValueError("An attached adapter cannot own the application clock")
            self.app = app
            cameras = app.env.renderer is not None
        elif backend == "simulation":
            profile = load_profile(robot, gripper=gripper, scene=scene)
            self.app = Application(recordings, profile=profile, cameras=cameras)
        elif backend == "serial":
            if robot != "parol6" or cameras:
                raise ValueError("Serial supports PAROL6 without camera streams")
            self.paced = False
            profile = load_profile(robot, gripper=gripper, scene="robot_only")
            self.app = SerialApplication(profile, port, tool_isolated)
        else:
            raise ValueError(f"Unknown backend: {backend}")

        if self.server_lock is not None:
            describe_owner(self.server_lock, robot=robot, gripper=gripper,
                           backend=backend, serial_port=port, scene=scene,
                           instance=self.app.state().instance)
        self.use_sim_clock = not self.serial and self.app.profile.task != "robot_only"

        try:
            self.configure_topics()
            if self.paced:
                self.app.start(paced=True)
            if backend == "simulation" and cameras:
                if self.paced:
                    self.worker = ObservationWorker(self.app.env)
                self.visualizer = Visualization(self.app.env, self.latest_observation)
            self.create_timer(0.05, self.publish_status)
            self.create_timer(0.05, self.check_source)
            self.get_logger().info(
                f"{robot} {backend} server ready; execution is paused"
            )
        except Exception:
            if self.owns_application:
                self.app.close()
            raise

    def configure_topics(self):
        self.spec = self.app.profile.spec
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        feedback_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE) if self.serial else qos
        self.states = self.create_publisher(JointState, "/joint_states", feedback_qos)
        self.statuses = self.create_publisher(Status, "/robot/status", qos)
        self.clock = (
            self.create_publisher(Clock, "/clock", qos)
            if self.use_sim_clock
            else None
        )
        self.references = self.create_publisher(JointState, "/robot/reference", feedback_qos)
        self.tracking = self.create_publisher(
            TrackingError, "/robot/tracking_error", QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.timing = self.create_publisher(Float64MultiArray, "/robot/timing", qos)
        self.ownership = self.create_publisher(String, "/robot/ownership", qos)
        self.serial_telemetry = None
        if self.serial:
            self.serial_telemetry = self.create_publisher(
                String, "/robot/serial_telemetry", qos
            )
        self.create_subscription(Control, "/robot/control", self.control, 10)
        self.create_subscription(JointGoal, "/robot/joint_goal", self.goal, qos)

    def latest_observation(self):
        if self.worker is not None:
            return self.worker.latest
        return self.app.current

    def status(self, state=None):
        if state is None:
            state = self.app.state()
        return Status(
            mode=state.mode,
            instance=state.instance,
            reason=state.reason,
            source=state.source,
            session=state.session,
            sequence=state.sequence,
            delivered_sequence=state.delivered_sequence,
            episode=state.episode,
            simulation_time_ns=state.time_ns,
            execution_time_ns=state.execution_time_ns,
            action_spec_id=self.spec.identity,
            joints=list(self.spec.joints),
            measured_positions=state.position,
            measured_velocities=state.velocity,
            commanded_reference=state.reference,
            recording=state.recording,
            command_id=self.command_id,
            command_result=self.command_result,
            command_accepted=self.command_accepted,
            motion_execution=state.motion_execution,
            motion_id=state.motion_id,
            motion_elapsed_seconds=state.motion_elapsed_seconds,
            motion_complete=state.motion_complete,
        )

    def packet(self, value):
        status = self.status()
        if (
            value.instance != status.instance
            or value.action_spec_id != self.spec.identity
        ):
            raise ValueError(
                "Joint command belongs to a different server or joint contract"
            )
        if (
            not value.session
            or value.session != status.session
            or value.sequence <= status.sequence
        ):
            raise ValueError(
                "Joint command belongs to an expired or reordered source session"
            )
        if value.motion_kind not in (JointGoal.SAMPLED, JointGoal.JOINT_MOVE, JointGoal.JOINT_PATH,
                                      JointGoal.LIVE_TARGET, JointGoal.HEARTBEAT):
            raise ValueError("Unknown motion kind")
        if value.motion_kind and not self.serial:
            raise ValueError("This backend requires sampled positions")
        if value.motion_kind == JointGoal.HEARTBEAT:
            if value.positions or value.velocities:
                raise ValueError("A motion heartbeat must not contain joint commands")
        else:
            self.spec.validate(value.positions)
        if value.velocities and (
            len(value.velocities) != len(self.spec.joints)
            or not all(math.isfinite(v) for v in value.velocities)
        ):
            raise ValueError("Feed-forward velocities require one finite value per joint")

    def goal(self, message):
        # Traffic for another instance/session is not a command to this robot.
        state = self.app.state()
        if message.instance != state.instance or not state.session or message.session != state.session:
            return
        try:
            self.packet(message)
            if self.serial:
                result = self.app.submit(message.positions, message.session, message.sequence,
                                         velocities=message.velocities,
                                         motion_kind=message.motion_kind, motion_id=message.motion_id,
                                         duration=message.duration, smoothing_period=message.smoothing_period,
                                         trajectory_times=message.trajectory_times,
                                         trajectory_positions=message.trajectory_positions)
            else:
                # The simulation's position actuators follow the sampled positions.
                result = self.app.submit(message.positions, message.session, message.sequence)
            if isinstance(result, CommandResult) and not result.accepted:
                self.app.pause("joint_command_rejected")
                self.get_logger().error(result.message)
                self.publish_status()
        except Exception as error:
            self.app.pause(f"joint_command_error: {error}")
            self.get_logger().error(
                f"Joint command rejected; execution paused: {error}"
            )
            self.publish_status()
        # Successful goals arrive at 100 Hz. Feedback and visualization belong
        # to the existing 20 Hz timer, not the motion-command callback.

    def check_source(self):
        if self.worker is not None and self.worker.failure:
            self.app.pause(f"camera_acquisition_failed: {self.worker.failure}")
            return
        status = self.status()
        if not status.session or status.mode not in ("Ready", "Running"):
            return
        publishers = self.get_publishers_info_by_topic("/robot/joint_goal")
        source_names = {
            info.node_namespace.rstrip("/") + "/" + info.node_name
            for info in publishers
        }
        if status.source not in source_names:
            self.app.pause("source_disconnected")
            self.publish_status()

    def control(self, message):
        if message.instance != self.app.state().instance:
            return
        if not message.command_id or message.command_id in self.seen_commands:
            return
        self.seen_commands.append(message.command_id)
        self.command_id = message.command_id
        self.command_accepted = False
        try:
            status = self.status()
            if message.instance != status.instance:
                raise ValueError(
                    "Control command belongs to an earlier server instance"
                )
            if message.episode != status.episode or message.session != status.session:
                raise ValueError(
                    "Control command belongs to an earlier episode or source session"
                )
            if message.operation == "activate":
                result = self.app.activate(message.source)
            else:
                if message.operation == "record" and self.paced:
                    raise ValueError("Recording requires stepped execution intervals")
                result = self.app.command(message.operation)
                if message.operation == "reset" and self.paced:
                    self.app.start(paced=True)
            if isinstance(result, CommandResult):
                self.command_result = result.message
                self.command_accepted = result.accepted
            else:
                self.command_result = json.dumps(result)
                self.command_accepted = True
        except Exception as error:
            self.command_result = str(error)
            self.get_logger().warning(f"Control {message.operation} rejected: {error}")
        self.publish_status()

    @staticmethod
    def stamp(time_ns, header):
        header.stamp.sec, header.stamp.nanosec = divmod(time_ns, 1_000_000_000)

    def publish_status(self):
        self.app.poll()
        state = self.app.state()
        status = self.status(state)
        if self.serial and state.mode != self.last_backend_mode:
            self.last_backend_mode = state.mode
            if state.mode == "Fault":
                self.get_logger().error(f"Serial fault; disabling: {state.reason}")
            else:
                self.get_logger().info(f"Serial state: {state.mode}")
        self.statuses.publish(status)
        if self.serial:
            # Drain every native sample once. Its capture stamp remains intact
            # even if this ROS callback publishes several samples in one batch.
            for sample in self.app.feedback_states():
                self.publish_feedback(sample)
        else:
            self.publish_feedback(state)
        self.timing.publish(
            Float64MultiArray(data=[state.time_ns / 1e9, state.execution_time_ns / 1e9,
                                   state.wall_lag_seconds])
        )
        ownership = dict(mode=state.mode, source=state.source, session=state.session,
                         reason=state.reason, feedback_kind=state.feedback_kind)
        self.ownership.publish(String(data=json.dumps(ownership)))
        if self.serial_telemetry is not None:
            telemetry = self.app.telemetry()
            self.log_estop_state(telemetry)
            self.serial_telemetry.publish(String(data=json.dumps(telemetry)))

    def publish_feedback(self, state):
        joints = JointState(
            name=list(self.spec.joints),
            position=state.position,
            velocity=state.velocity,
        )
        joint_time_ns = (state.time_ns if self.use_sim_clock else
                         state.sample_wall_time_ns if self.serial else time.time_ns())
        self.stamp(joint_time_ns, joints.header)
        if len(state.position) == len(self.spec.joints):
            self.states.publish(joints)
        self.references.publish(
            JointState(header=joints.header, name=joints.name, position=state.reference)
        )
        if len(state.reference) == len(state.position):
            errors = [
                goal - actual for goal, actual in zip(state.reference, state.position)
            ]
            self.tracking.publish(TrackingError(header=joints.header, data=errors,
                                                reference_valid=state.reference_valid,
                                                timing_uncertainty_seconds=state.timing_uncertainty_seconds))
        if self.clock is not None:
            clock = Clock()
            clock.clock = joints.header.stamp
            self.clock.publish(clock)

    def log_estop_state(self, telemetry):
        if not telemetry["connected"]:
            self.last_estop_state = None
            return
        state = (telemetry["estop"], telemetry["estop_verified"])
        if state == self.last_estop_state:
            return
        self.last_estop_state = state
        if state[0]:
            self.get_logger().warning("E-stop PRESSED: input detected")
        elif state[1]:
            self.get_logger().info("E-stop RELEASED: press/release verified")
        else:
            self.get_logger().info("E-stop released; press and release once before homing")

    def destroy_node(self):
        try:
            if self.owns_application:
                self.app.pause("server_shutdown")
        finally:
            try:
                if self.visualizer is not None:
                    self.visualizer.destroy_node()
                if self.worker is not None:
                    self.worker.close()
            finally:
                try:
                    if self.owns_application:
                        self.app.close()
                finally:
                    if self.server_lock is not None:
                        self.server_lock.close()
                    super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Adapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        # The process boundary owns unexpected failures; cleanup always disables IO.
        rclpy.logging.get_logger("robot_server").error(f"Server stopped: {error}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
