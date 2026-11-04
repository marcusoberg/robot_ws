#!/usr/bin/env python3
"""ROS ownership and unit conversion around the upstream PAROL6 Python API."""

from collections import deque
import json
import secrets
import time
import uuid

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_description.spec import parol6
from robot_server.msg import Control, JointGoal, Status
from robot_server_core.controller import Parol6Controller
from robot_server_node.ownership import acquire_server_lock, describe_owner


class SerialNode(Node):
    def __init__(self):
        super().__init__('robot_server', start_parameter_services=False)
        self.server_lock = acquire_server_lock()
        serial_port = self.declare_parameter('serial_port', '').value
        if not serial_port:
            raise ValueError('serial_port must explicitly name the PAROL6 device')
        api_port = self.declare_parameter('api_port', 5001).value
        gripper = self.declare_parameter('gripper', 'pneumatic').value
        self.profile = parol6(gripper, scene='robot_only')
        self.spec = self.profile.spec
        self.instance = uuid.uuid4().hex
        self.episode = 0
        self.session = 0
        self.source = ''
        self.sequence = 0
        self.last_goal_at = 0.
        self.mode = 'Idle'
        self.reason = ''
        self.command_id = ''
        self.command_result = ''
        self.command_accepted = False
        self.commands_seen = deque(maxlen=128)
        self.pending_controls = []
        self.stream_sends = []
        self.motion_id = 0
        self.motion_future = None
        self.motion_complete = False
        self.motion_started_at = 0.
        self.homing = False
        self.last_feedback_key = None
        self.reference = np.asarray(self.profile.initial).copy()
        self.last_backend_mode = None
        describe_owner(self.server_lock, robot='parol6', gripper=gripper, backend='serial',
                       scene='robot_only', serial_port=serial_port, instance=self.instance)
        self.controller = Parol6Controller(serial_port, api_port)
        self.statuses = self.create_publisher(Status, '/robot/status', 1)
        self.states = self.create_publisher(JointState, '/joint_states', 10)
        self.telemetry = self.create_publisher(String, '/robot/serial_telemetry', 1)
        self.create_subscription(Control, '/robot/control', self.on_control, 10)
        self.create_subscription(JointGoal, '/robot/joint_goal', self.on_joint_goal, 1)
        self.create_timer(.02, self.update)
        self.get_logger().info('PAROL6 Python API server started; homing and activation are explicit.')

    def acknowledge(self, message, accepted, detail):
        self.command_id = message.command_id
        self.command_accepted = accepted
        self.command_result = detail

    def release(self):
        self.session = 0
        self.source = ''
        self.sequence = 0
        self.motion_id = 0
        self.motion_complete = False

    def fault(self, reason):
        if self.mode != 'Fault':
            self.get_logger().error(str(reason))
            self.controller.stop()
        self.mode, self.reason = 'Fault', str(reason)
        self.release()

    def on_control(self, message):
        if message.command_id in self.commands_seen:
            return
        self.commands_seen.append(message.command_id)
        if message.instance != self.instance or message.episode != self.episode:
            self.acknowledge(message, False, 'Command belongs to an earlier server instance or episode')
            return
        if message.source and message.session != self.session:
            self.acknowledge(message, False, 'Command belongs to an earlier source session')
            return
        operation = message.operation
        if operation == 'activate':
            if self.mode != 'Ready' or self.session or not message.source:
                self.acknowledge(message, False, f'Server is not available: {self.mode}')
                return
            self.session = secrets.randbits(63) or 1
            self.source = message.source
            self.sequence = 0
            self.last_goal_at = time.monotonic()
            self.mode = 'Running'
            self.acknowledge(message, True, f'Motion session {self.session} started')
        elif operation == 'home':
            if self.session or self.homing:
                self.acknowledge(message, False, 'Pause the active source before homing')
                return
            self.reason = ''
            self.homing = True
            self.mode = 'Homing'
            self.motion_future = self.controller.home()
            self.acknowledge(message, True, 'Homing requested from the Python API')
        elif operation in ('pause', 'disable', 'enable', 'reset'):
            self.release()
            self.homing = False
            self.motion_future = None
            self.reason = ''
            if operation == 'disable':
                future = self.controller.disable()
            elif operation in ('enable', 'reset'):
                future = self.controller.enable()
            else:
                future = self.controller.stop()
            self.mode = 'Stopping'
            self.pending_controls.append((message, future))
        else:
            self.acknowledge(message, False, f'{operation} is not implemented in the Python API experiment')

    def on_joint_goal(self, message):
        if (message.instance != self.instance or not self.session
                or message.session != self.session or message.sequence <= self.sequence):
            return
        if message.action_spec_id != self.spec.identity:
            self.fault('Source joint contract differs from the server')
            return
        self.sequence = message.sequence
        self.last_goal_at = time.monotonic()
        if message.motion_kind == JointGoal.HEARTBEAT:
            return
        try:
            target = self.spec.validate(message.positions)
            degrees = np.rad2deg(target[:6]).tolist()
            self.reference = target
            if message.motion_kind == JointGoal.JOINT_MOVE:
                if message.motion_id == self.motion_id:
                    return
                if not np.isfinite(message.duration) or message.duration <= 0:
                    raise ValueError('Move duration must be positive and finite')
                self.motion_id = message.motion_id
                self.motion_complete = False
                self.motion_started_at = time.monotonic()
                self.motion_future = self.controller.move(degrees, message.duration)
            elif message.motion_kind in (JointGoal.SAMPLED, JointGoal.LIVE_TARGET):
                if message.motion_id != self.motion_id:
                    self.motion_started_at = time.monotonic()
                self.motion_id = message.motion_id
                self.motion_complete = False
                self.stream_sends.append(self.controller.servo(degrees))
            else:
                raise ValueError('Unknown motion kind')
        except ValueError as error:
            self.fault(error)

    def collect_commands(self):
        pending = []
        for message, future in self.pending_controls:
            if not future.done():
                pending.append((message, future))
                continue
            try:
                accepted = future.result() == 1
                self.acknowledge(message, accepted, f'{message.operation}: ' + ('accepted' if accepted else 'not acknowledged'))
                self.mode = 'Idle'
                if not accepted:
                    self.fault(self.command_result)
            except Exception as error:
                self.acknowledge(message, False, str(error))
                self.fault(error)
        self.pending_controls = pending
        pending = []
        for future in self.stream_sends:
            if future.done():
                try:
                    if future.result() != 1:
                        raise RuntimeError('Python API did not send the servo target')
                except Exception as error:
                    self.fault(error)
            else:
                pending.append(future)
        self.stream_sends = pending
        if self.motion_future is not None and self.motion_future.done():
            future, self.motion_future = self.motion_future, None
            try:
                if future.result() < 0:
                    raise RuntimeError('Python API rejected the motion')
                self.motion_complete = True
                if self.homing:
                    self.homing = False
                    self.episode += 1
                    self.mode = 'Idle'
            except Exception as error:
                self.homing = False
                self.fault(error)

    def update(self):
        self.collect_commands()
        if self.controller.failure is not None:
            self.fault(self.controller.failure)
        snapshot = self.controller.snapshot
        if snapshot is None:
            return
        feedback, received_at = snapshot
        now = time.monotonic()
        if now - received_at > .5:
            self.fault('Python API feedback expired')
        if self.session and now - self.last_goal_at > self.profile.source_timeout_ns / 1e9:
            self.controller.stop()
            self.release()
            self.mode, self.reason = 'Stopping', 'Source feedback expired'
        if feedback.io[4] == 0:
            self.fault('Physical E-stop asserted')
        elif feedback.error is not None:
            self.fault(str(feedback.error))
        elif self.mode != 'Fault' and not self.pending_controls:
            if self.homing:
                self.mode = 'Homing'
            elif not feedback.enabled or not feedback.homed:
                self.mode = 'Idle'
                self.reason = 'Homing required' if not feedback.homed else 'Controller disabled'
                self.release()
            elif self.mode == 'Stopping' and np.max(np.abs(feedback.speeds)) > 0.5:
                pass
            else:
                self.mode = 'Running' if self.session else 'Ready'
                self.reason = ''
        positions = np.r_[np.deg2rad(feedback.angles), self.reference[-1]].tolist()
        velocities = np.r_[np.deg2rad(feedback.speeds), 0.].tolist()
        self.statuses.publish(Status(
            mode=self.mode, instance=self.instance, episode=self.episode, reason=self.reason,
            source=self.source, session=self.session, sequence=self.sequence, delivered_sequence=0,
            execution_time_ns=feedback.mono_time_ns, action_spec_id=self.spec.identity,
            joints=list(self.spec.joints), measured_positions=positions, measured_velocities=velocities,
            commanded_reference=self.reference.tolist(), command_id=self.command_id,
            command_result=self.command_result, command_accepted=self.command_accepted,
            motion_execution='python_api', motion_id=self.motion_id, motion_complete=self.motion_complete,
            motion_elapsed_seconds=now - self.motion_started_at if self.motion_id else 0.,
            standby_positions=np.deg2rad(self.controller.standby_degrees).tolist(),
        ))
        key = feedback.session_id, feedback.seq
        if key != self.last_feedback_key:
            self.last_feedback_key = key
            stamp = self.get_clock().now().to_msg()
            joints = JointState(name=list(self.spec.joints), position=positions, velocity=velocities)
            joints.header.stamp = stamp
            self.states.publish(joints)
            self.telemetry.publish(String(data=json.dumps(dict(
                enabled=feedback.enabled, homed=feedback.homed, estop=bool(feedback.io[4] == 0),
                controller_session=feedback.session_id, sequence=feedback.seq,
                controller_time_ns=feedback.mono_time_ns,
            ))))
        if (self.mode, self.reason) != self.last_backend_mode:
            self.last_backend_mode = self.mode, self.reason
            self.get_logger().info(f'{self.mode}: {self.reason}')

    def destroy_node(self):
        try:
            self.controller.close()
        finally:
            self.server_lock.close()
            super().destroy_node()


def main():
    rclpy.init()
    node = SerialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
