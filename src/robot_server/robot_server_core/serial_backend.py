"""Robot-server ownership of the stock PAROL6 serial runtime.

No geometry or IK is implemented here. Producers submit joint goals through the
same ROS interface used by simulation. The native thread owns serial IO, E-stop
monitoring and motion deadlines, including while Python or ROS is delayed.
"""

import secrets
import math
import time
import uuid
from collections import deque

from robot_runtime_native._native import Parol6Serial
from robot_server_core.state import CommandResult, ServerState
from robot_server_core.motion_trace import MotionTrace


class SerialApplication:
    def __init__(self, profile, port, tool_isolated):
        if not port:
            raise ValueError("The serial backend requires an explicit serial_port")
        if not tool_isolated:
            raise ValueError(
                "Set tool_isolated:=true only with the tool unloaded/isolated; outputs go LOW"
            )
        expected_joints = ("L1", "L2", "L3", "L4", "L5", "L6", "gripper_slide")
        if profile.spec.joints != expected_joints:
            raise ValueError("Serial PAROL6 requires the stock six-joint ordering and passive gripper")
        profile.spec.validate(profile.initial)
        self.profile = profile
        self.port = port
        self.instance = uuid.uuid4().hex
        self.episode = 1
        self.session = 0
        self.source = ""
        self.sequence = 0
        self.reference = []
        self.motion_id = 0
        self.motion_request = None
        self.feedback_queue = deque(maxlen=512)
        self.motion_trace = MotionTrace('server', profile.spec.joints)
        self.transport = Parol6Serial(port)

    def poll(self):
        # Ready also holds enabled motors; loss of the ROS owner must disable them.
        # The native method never renews a Running motion-command lease.
        self.transport.keepalive()
        self.capture_feedback()

    def capture_feedback(self):
        for feedback in self.transport.drain_samples():
            self.feedback_queue.append(feedback)
            self.motion_trace.record(
                'serial_sample', frame=feedback.frames, mode=feedback.mode,
                sample_time_ns=feedback.sample_time_ns, receive_time_ns=feedback.receive_time_ns,
                transmit_time_ns=feedback.transmit_time_ns,
                write_complete_time_ns=feedback.write_complete_time_ns,
                evaluation_time_ns=feedback.evaluation_time_ns,
                reference_frame=feedback.reference_frame, reference_time_ns=feedback.reference_time_ns,
                transmitted_motion_id=feedback.transmitted_motion_id,
                reference_motion_id=feedback.reference_motion_id,
                round_trip_seconds=feedback.round_trip_seconds,
                timing_uncertainty_seconds=feedback.timing_uncertainty_seconds,
                transmitted_position=list(feedback.transmitted_position),
                transmitted_velocity=list(feedback.transmitted_velocity),
                reference=list(feedback.reference), reference_valid=feedback.reference_valid,
                measured=list(feedback.position), velocity=list(feedback.velocity),
                motion_id=feedback.motion_id, motion_elapsed_seconds=feedback.motion_elapsed_seconds,
                firmware_timing_ticks=feedback.timing_ticks,
            )

    def feedback_states(self):
        samples = list(self.feedback_queue)
        self.feedback_queue.clear()
        return [self.make_state(sample) for sample in samples]

    def state(self):
        telemetry = self.transport.observe()
        if telemetry.mode == 'Fault':
            self.capture_feedback()
            self.motion_trace.save(telemetry.reason)
        if telemetry.mode in ("Fault", "Closed", "Stopped"):
            self.session = 0
            self.source = ""
        return self.make_state(telemetry)

    def make_state(self, telemetry):
        positions = list(telemetry.position)
        velocities = list(telemetry.velocity)
        if positions:
            # The stock arm packet has no measurement for this passive URDF joint.
            positions.append(self.profile.initial[-1])
            velocities.append(0.0)
        reference = list(telemetry.reference)
        if reference:
            reference.append(self.profile.initial[-1])
        else:
            reference = positions
        reason = telemetry.reason
        if telemetry.mode == "Fault":
            path = self.motion_trace.path
            if path:
                reason = f"{reason}; command trace: {path}"
        if telemetry.mode == "Idle":
            if telemetry.homed_this_session:
                reason = "Controller retains homing and joint counters; run 'pixi run enable'"
            elif telemetry.estop_verified:
                reason = "E-stop input verified; homing required"
            else:
                reason = (
                    "Press and release E-stop while disabled before requesting home"
                )
        return ServerState(
            mode=telemetry.mode,
            instance=self.instance,
            episode=self.episode,
            time_ns=telemetry.sample_time_ns,
            execution_time_ns=telemetry.transmit_time_ns,
            position=positions,
            velocity=velocities,
            reference=list(reference),
            session=self.session,
            source=self.source,
            sequence=self.sequence,
            reason=reason,
            wall_lag_seconds=max(0., (time.monotonic_ns()-telemetry.sample_time_ns)/1e9),
            feedback_kind="step_count_estimate; host-estimated sample time; gripper unmeasured",
            sample_wall_time_ns=telemetry.sample_wall_time_ns,
            reference_valid=telemetry.reference_valid,
            timing_uncertainty_seconds=telemetry.timing_uncertainty_seconds,
            motion_execution="native_serial",
            motion_id=telemetry.motion_id,
            motion_elapsed_seconds=telemetry.motion_elapsed_seconds,
            motion_complete=telemetry.motion_complete,
        )

    def telemetry(self):
        state = self.transport.observe()
        fields = (
            "mode",
            "reason",
            "connected",
            "age_seconds",
            "estop",
            "estop_verified",
            "homed_this_session",
            "steps",
            "speed_steps",
            "homed_bits",
            "io_bits",
            "temperature_bits",
            "position_error_bits",
            "timing_ticks",
            "timeout_value",
            "command_echo",
            "gripper_id",
            "gripper_position",
            "gripper_speed",
            "gripper_current",
            "gripper_status",
            "object_detection",
            "payload",
            "reference", "reference_valid", "transmitted_position", "transmitted_velocity",
            "sample_time_ns", "receive_time_ns", "transmit_time_ns", "round_trip_seconds",
            "timing_uncertainty_seconds", "cycle_seconds", "goal_interval_seconds",
            "motion_id", "motion_elapsed_seconds", "motion_complete", "reference_frame",
            "write_complete_time_ns", "evaluation_time_ns", "reference_time_ns",
            "transmitted_motion_id", "reference_motion_id",
        )
        result = {field: getattr(state, field) for field in fields}
        result['sample_time_basis'] = 'host exchange midpoint; firmware snapshot/apply latency unknown'
        return result

    def result(self, accepted, message):
        if not accepted:
            telemetry = self.transport.observe()
            message = telemetry.command_error or telemetry.reason or message
        return CommandResult(accepted, message)

    def activate(self, source):
        if self.session or not source:
            return CommandResult(False, "Activation requires an unowned server and source identity")
        if not self.transport.start():
            return self.result(False, "Activation rejected")
        self.session = secrets.randbits(63) or 1
        self.motion_trace.reset()
        self.motion_trace.record('activate', session=self.session)
        self.source = source
        self.sequence = 0
        self.motion_id = 0
        self.motion_request = None
        self.reference = self.state().position
        return CommandResult(True, f"Motion session {self.session} started")

    def submit(self, goal, session, sequence, velocities=(), *, motion_kind=0, motion_id=0,
               duration=0., smoothing_period=0., trajectory_times=(), trajectory_positions=()):
        feedback = self.transport.observe()
        self.motion_trace.record(
            'goal', session=int(session), sequence=int(sequence),
            positions=list(goal), velocities=list(velocities),
            feedback=list(feedback.position), feedback_velocity=list(feedback.velocity),
            feedback_age=feedback.age_seconds, serial_frames=feedback.frames,
            serial_cycle_seconds=feedback.cycle_seconds,
            previous_goal_interval_seconds=feedback.goal_interval_seconds,
            firmware_timing_ticks=feedback.timing_ticks, motion_kind=motion_kind, motion_id=motion_id,
        )
        # ROS has checked ownership, ordering and ActionSpec at its input boundary.
        # The native boundary independently enforces limits for every caller.
        if not session or session != self.session or sequence <= self.sequence:
            return CommandResult(False, "Joint goal belongs to an expired or reordered session")
        if motion_kind != 4 and (len(goal) != 7 or goal[-1] != self.profile.initial[-1]):
            self.transport.stop()
            return CommandResult(False, "Serial gripper actuation is not implemented")
        if velocities and (len(velocities) != 7 or velocities[-1] != 0
                           or not all(math.isfinite(v) for v in velocities)):
            self.transport.stop()
            return CommandResult(False, "Expected finite arm velocities and zero gripper velocity")
        if motion_kind in (1, 2):
            # Retries acknowledge the same immutable plan without resetting time.
            request = (motion_kind, tuple(goal), duration, tuple(trajectory_times), tuple(trajectory_positions))
            if motion_id == self.motion_id:
                if request != self.motion_request:
                    return CommandResult(False, "Motion id reused with a different trajectory")
                accepted = self.transport.heartbeat(motion_id)
            elif motion_id <= self.motion_id:
                return CommandResult(False, "Trajectory motion id is stale")
            elif motion_kind == 1:
                accepted = self.transport.move_joint(list(goal[:6]), duration, motion_id)
            else:
                if (len(trajectory_times) < 2 or
                        len(trajectory_positions) != 7*len(trajectory_times)):
                    return CommandResult(False, "Path requires one complete joint row per time")
                arm = []
                for offset in range(0, len(trajectory_positions), 7):
                    row = trajectory_positions[offset:offset+7]
                    self.profile.spec.validate(row)
                    if row[-1] != self.profile.initial[-1]:
                        return CommandResult(False, "Serial path cannot actuate the gripper")
                    arm.extend(row[:6])
                accepted = self.transport.follow_path(list(trajectory_times), arm, motion_id)
            if accepted:
                self.motion_request = request
                self.motion_id = motion_id
        elif motion_kind == 3:
            if motion_id < self.motion_id:
                return CommandResult(False, "Live motion id is stale")
            accepted = self.transport.live_target(list(goal[:6]), smoothing_period, motion_id)
            if accepted:
                self.motion_id = motion_id
                self.motion_request = None
        elif motion_kind == 4:
            accepted = self.transport.heartbeat(motion_id)
        elif motion_kind == 0:
            accepted = self.transport.submit(list(goal[:6]), list(velocities[:6]))
        else:
            return CommandResult(False, "Unknown motion kind")
        if not accepted:
            return self.result(False, "Joint goal rejected")
        self.sequence = sequence
        return CommandResult(True, "Joint goal accepted")

    def pause(self, reason="paused"):
        if reason in ("paused", "pause"):
            self.transport.pause()
        else:
            self.transport.stop()
        self.session = 0
        self.source = ""
        state = self.state()
        self.reference = state.position
        return state

    def command(self, operation):
        if operation == "home":
            return self.result(self.transport.home(), "Homing started; waiting for completion")
        if operation == "enable":
            return self.result(self.transport.enable(), "Enabling with the controller's retained joint position")
        if operation in ("pause", "disable"):
            return CommandResult(True, self.pause(operation).mode)
        if operation == "reset":
            self.transport.pause()
            if not self.transport.reset():
                return self.result(False, "Could not reset the connected controller")
            self.session = 0
            self.source = ""
            self.sequence = 0
            self.reference = []
            self.episode += 1
            self.instance = uuid.uuid4().hex
            if self.transport.observe().homed_this_session:
                return CommandResult(True, "Reset complete; restoring hold at the retained joint position")
            return CommandResult(True, "Reset complete; controller position invalid, homing required")
        return CommandResult(False, f"Serial backend does not support operation: {operation}")

    def close(self):
        try:
            self.transport.stop()
        finally:
            self.transport.close()
