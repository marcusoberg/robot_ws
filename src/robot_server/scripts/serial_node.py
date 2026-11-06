#!/usr/bin/env python3
"""ROS ownership and unit conversion around the upstream PAROL6 Python API."""

from collections import deque
import json
import secrets
import time
import uuid

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_core.control.python_api import PythonApiController
from robot_core.config import robot_settings
from robot_core.kinematics.chain import load_chain
from robot_core.kinematics.servo import CartesianServo
from robot_server_client.poses import pose_values, stamped_pose
from robot_core.validation import ros_joint_target
from robot_core.data.commands import MoveJoint
from robot_description.spec import parol6
from robot_server.msg import Control, JointGoal, PoseGoal, Status
from robot_server_node.ownership import acquire_server_lock, describe_owner


class SerialNode(Node):
    def __init__(self):
        super().__init__('robot_server', start_parameter_services=False)
        self.server_lock = acquire_server_lock()
        hardware = robot_settings()['hardware']
        self.motion = robot_settings()['motion']
        serial_port = self.declare_parameter('serial_port', hardware['serial_port']).value
        api_port = self.declare_parameter('api_port', hardware['api_port']).value
        gripper = self.declare_parameter('gripper', robot_settings()['robots']['parol6']['gripper']).value
        self.profile = parol6(gripper, scene='robot_only')
        self.spec = self.profile.spec
        tracking = robot_settings()['cartesian']
        self.base_frame, self.tip = tracking['base'], tracking['tip']
        chain = load_chain(self.profile.description, self.spec.joints, self.base_frame, self.tip)
        self.cartesian = CartesianServo(chain, tracking[gripper])
        self.pose_target = None
        self.last_pose_step = None
        self.starting_teleop = False
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
        self.commands_seen = deque(maxlen=self.motion['command_history'])
        self.pending_controls = []
        self.stream_sends = []
        self.motion_id = 0
        self.motion_future = None
        self.motion_finished_at = None
        self.motion_started_at = 0.
        self.homing = False
        self.last_feedback_key = None
        self.reference = np.asarray(self.profile.initial).copy()
        self.last_backend_mode = None
        describe_owner(self.server_lock, robot='parol6', gripper=gripper, backend='serial',
                       scene='robot_only', serial_port=serial_port, instance=self.instance)
        self.controller = PythonApiController(serial_port, api_port)
        self.statuses = self.create_publisher(Status, '/robot/status', robot_settings()["ros"]["latest_depth"])
        self.states = self.create_publisher(JointState, '/joint_states', robot_settings()["ros"]["command_depth"])
        self.telemetry = self.create_publisher(String, '/robot/serial_telemetry', robot_settings()["ros"]["latest_depth"])
        self.create_subscription(Control, '/robot/control', self.on_control, robot_settings()["ros"]["command_depth"])
        self.create_subscription(JointGoal, '/robot/joint_goal', self.on_joint_goal, robot_settings()["ros"]["latest_depth"])
        self.create_subscription(PoseGoal, '/robot/pose_goal', self.on_pose_goal, robot_settings()['ros']['latest_depth'])
        self.create_timer(self.motion['status_period'], self.update)
        self.get_logger().info('PAROL6 Python API server started; homing and activation are explicit.')

    def acknowledge(self, message, accepted, detail):
        self.command_id = message.command_id
        self.command_accepted = accepted
        self.command_result = detail

    def release(self):
        if self.session and self.motion_future is not None:
            self.motion_future.cancel()
            self.motion_future = None
        self.cartesian.anchor = None
        self.pose_target = None
        self.last_pose_step = None
        self.starting_teleop = False
        self.session = 0
        self.source = ''
        self.sequence = 0
        self.motion_id = 0
        self.motion_finished_at = None

    def fault(self, reason):
        if self.mode != 'Fault':
            self.get_logger().error(str(reason))
            self.controller.stop()
        self.mode, self.reason = 'Fault', str(reason)
        self.release()

    def on_control(self, message):
        if message.operation == 'heartbeat':
            # Startup owns a timed move. Only its source may renew that lease.
            if (self.starting_teleop and self.session and message.session == self.session
                    and message.instance == self.instance and message.episode == self.episode
                    and message.source == self.source):
                self.last_goal_at = time.monotonic()
            return
        if message.command_id in self.commands_seen:
            return
        self.commands_seen.append(message.command_id)
        if message.instance != self.instance or message.episode != self.episode:
            self.acknowledge(message, False, 'Command belongs to an earlier server instance or episode')
            return
        if message.source and (message.session != self.session or (self.session and message.source != self.source)):
            self.acknowledge(message, False, 'Command belongs to an earlier source session')
            return
        operation = message.operation
        if operation in ('activate', 'activate_teleop'):
            if self.mode != 'Ready' or self.session or not message.source:
                self.acknowledge(message, False, f'Server is not available: {self.mode}')
                return
            self.session = secrets.randbits(63) or 1
            self.source = message.source
            self.sequence = 0
            self.last_goal_at = time.monotonic()
            self.mode = 'Running'
            if operation == 'activate_teleop':
                self.starting_teleop = True
                self.motion_finished_at = None
                self.reference[:6] = np.deg2rad(self.motion['pose_degrees'])
                self.motion_future = self.controller.execute(MoveJoint(self.reference[:6], self.motion['duration']))
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

    def owns_goal(self, message):
        if (message.instance != self.instance or not self.session
                or message.session != self.session or message.sequence <= self.sequence):
            return False
        if message.action_spec_id != self.spec.identity:
            self.fault('Source joint contract differs from the server')
            return False
        return True

    def on_pose_goal(self, message):
        if not self.owns_goal(message) or message.episode != self.episode or self.motion_future is not None:
            return
        target = message.target
        stamp = target.header.stamp
        age = self.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)
        if not 0 <= age < self.profile.source_timeout_ns:
            return
        if target.header.frame_id != self.base_frame or message.tip != self.tip:
            self.fault(f'Pose target must specify {self.tip} in {self.base_frame}')
            return
        xyz, xyzw = pose_values(target)
        norm = np.linalg.norm(xyzw)
        if not np.isfinite(xyz).all() or not np.isfinite(norm) or norm == 0:
            self.fault('Invalid Cartesian pose')
            return
        if message.gripper_position:
            # Physical PAROL6 gripper actuation is not provided by this backend.
            self.fault('The serial Cartesian interface supports arm motion only')
            return
        rotation = pin.Quaternion(*(xyzw[[3, 0, 1, 2]] / norm)).matrix()
        self.pose_target = pin.SE3(rotation, xyz), message.position_only
        self.motion_finished_at = None
        self.starting_teleop = False
        self.sequence = message.sequence
        self.last_goal_at = time.monotonic() - age / 1e9

    def on_joint_goal(self, message):
        if not self.owns_goal(message):
            return
        if self.pose_target is not None and message.motion_kind == JointGoal.HEARTBEAT:
            return  # A Cartesian stream requires fresh poses from its producer.
        if self.motion_future is not None and message.motion_kind != JointGoal.HEARTBEAT:
            return
        self.sequence = message.sequence
        self.last_goal_at = time.monotonic()
        if message.motion_kind == JointGoal.HEARTBEAT:
            return
        self.pose_target = None
        self.last_pose_step = None
        self.cartesian.anchor = None
        self.starting_teleop = False
        try:
            target = ros_joint_target(self.spec, message.positions)
            self.reference = target
            if message.motion_kind == JointGoal.JOINT_MOVE:
                if message.motion_id == self.motion_id:
                    return
                command = MoveJoint(target[:6], message.duration)
                self.motion_id = message.motion_id
                self.motion_finished_at = None
                self.motion_started_at = time.monotonic()
                self.motion_future = self.controller.execute(command)
            elif message.motion_kind in (JointGoal.SAMPLED, JointGoal.LIVE_TARGET):
                if message.motion_id != self.motion_id:
                    self.motion_started_at = time.monotonic()
                self.motion_id = message.motion_id
                self.motion_finished_at = None
                self.stream_sends.append(self.controller.servo(np.rad2deg(target[:6]).tolist()))
            else:
                self.fault(f'Unknown ROS motion kind {message.motion_kind}')
        except ValueError as error:
            self.fault(error)

    def collect_commands(self):
        pending = []
        for message, future in self.pending_controls:
            if not future.done():
                pending.append((message, future))
                continue
            try:
                future.result()
                self.acknowledge(message, True, f'{message.operation}: accepted')
                self.mode = 'Idle'
            except Exception as error:
                self.acknowledge(message, False, str(error))
                self.fault(error)
        self.pending_controls = pending
        pending = []
        for future in self.stream_sends:
            if future.done():
                try:
                    future.result()
                except Exception as error:
                    self.fault(error)
            else:
                pending.append(future)
        self.stream_sends = pending
        if self.motion_future is not None and self.motion_future.done():
            future, self.motion_future = self.motion_future, None
            try:
                future.result()
                self.motion_finished_at = time.monotonic()
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
        if now - received_at > self.motion['feedback_timeout']:
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
            elif self.mode == 'Stopping' and np.max(np.abs(feedback.speeds)) > robot_settings()['hardware']['stopped_speed_deg_s']:
                pass
            else:
                self.mode = 'Running' if self.session else 'Ready'
                self.reason = ''
        positions = np.r_[np.deg2rad(feedback.angles), self.reference[-1]].tolist()
        velocities = np.r_[np.deg2rad(feedback.speeds), 0.].tolist()
        key = feedback.session_id, feedback.seq
        tcp = self.cartesian.pose(positions)
        stamp = self.get_clock().now().to_msg()
        if self.mode == 'Running' and self.pose_target is not None and key != self.last_feedback_key and not self.stream_sends:
            target, position_only = self.pose_target
            period = self.motion['status_period']
            if self.last_pose_step is not None:
                period = min(period, now - self.last_pose_step)
            self.last_pose_step = now
            self.reference = self.cartesian.step(target, positions, period, position_only=position_only)
            self.stream_sends.append(self.controller.servo(np.rad2deg(self.reference[:6]).tolist()))
        self.statuses.publish(Status(
            tcp_link=self.tip, tcp_pose=stamped_pose(self.base_frame, stamp, tcp.translation, pin.Quaternion(tcp.rotation).coeffs()),
            mode=self.mode, instance=self.instance, episode=self.episode, reason=self.reason,
            source=self.source, session=self.session, sequence=self.sequence, delivered_sequence=0,
            execution_time_ns=feedback.mono_time_ns, action_spec_id=self.spec.identity,
            joints=list(self.spec.joints), measured_positions=positions, measured_velocities=velocities,
            commanded_reference=self.reference.tolist(), command_id=self.command_id,
            command_result=self.command_result, command_accepted=self.command_accepted,
            motion_execution='python_api', motion_id=self.motion_id,
            # Alignment must use controller feedback received after the move completed.
            motion_complete=self.motion_finished_at is not None and received_at >= self.motion_finished_at,
            motion_elapsed_seconds=now - self.motion_started_at if self.motion_id else 0.,
            standby_positions=np.deg2rad(self.controller.standby_degrees).tolist(),
        ))
        if key != self.last_feedback_key:
            self.last_feedback_key = key
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
