import math
import time
import uuid
from collections import deque

from rclpy.node import Node
from robot_server.msg import Control, JointGoal, Status


class Source(Node):
    def __init__(self, name, spec):
        super().__init__(f'{name}_{uuid.uuid4().hex[:8]}', start_parameter_services=False)
        self.spec = spec
        self.source = self.get_fully_qualified_name()
        self.status = None
        self.sent_sequence = 0
        self.session = 0
        self.pending_commands = deque(maxlen=32)
        self.motion_id = 0
        self.motion_request = None
        self.live_motion = False
        self.last_motion_sent_at = 0.
        self.commands = self.create_publisher(Control, '/robot/control', 10)
        self.goals = self.create_publisher(JointGoal, '/robot/joint_goal', 1)
        self.create_subscription(Status, '/robot/status', self.receive_status, 1)

    def receive_status(self, status):
        if status.action_spec_id != self.spec.identity:
            self.get_logger().error('Server action specification differs from this source')
            self.status = None
            return
        previous = self.status
        self.status = status
        for command_id, operation in tuple(self.pending_commands):
            if status.command_id == command_id:
                if status.command_accepted:
                    self.get_logger().info(f'{operation}: {status.command_result}')
                else:
                    self.get_logger().error(f'{operation}: {status.command_result}')
                self.pending_commands.remove((command_id, operation))
        session = status.session if status.source == self.source else 0
        if session != self.session or previous is None or previous.instance != status.instance:
            self.session = session
            self.sent_sequence = status.sequence if session else 0
            self.motion_request = None
            self.live_motion = False
            self.last_motion_sent_at = 0.
            self.session_changed()

    def session_changed(self):
        pass

    def control(self, operation):
        if self.status is None:
            self.get_logger().warning('Waiting for /robot/status')
            return
        command_id = uuid.uuid4().hex
        self.pending_commands.append((command_id, operation))
        self.commands.publish(Control(command_id=command_id, instance=self.status.instance,
                                      episode=self.status.episode, operation=operation, source=self.source,
                                      session=self.status.session))

    def toggle(self):
        self.control('pause' if self.session else 'activate')

    def available(self):
        return self.status is not None and self.session and self.status.mode in ('Ready', 'Running')

    @property
    def native_motion(self):
        return self.status is not None and self.status.motion_execution == 'native_serial'

    def begin_move(self, target, duration):
        self.motion_id += 1
        self.live_motion = False
        self.motion_request = dict(motion_kind=JointGoal.JOINT_MOVE, motion_id=self.motion_id,
                                   positions=list(target), duration=float(duration))
        return self.motion_heartbeat()

    def begin_path(self, times, positions):
        self.motion_id += 1
        self.live_motion = False
        self.motion_request = dict(motion_kind=JointGoal.JOINT_PATH, motion_id=self.motion_id,
                                   positions=list(positions[0]), trajectory_times=list(times),
                                   trajectory_positions=[float(q) for row in positions for q in row])
        return self.motion_heartbeat()

    def send_live(self, target, period):
        if not self.live_motion:
            self.motion_id += 1
            self.live_motion = True
        self.motion_request = dict(positions=list(target), motion_kind=JointGoal.LIVE_TARGET,
                                   motion_id=self.motion_id, smoothing_period=float(period))
        request = self.motion_request.copy()
        goal = request.pop('positions')
        return self.send(goal, **request)

    def motion_heartbeat(self):
        if not self.available():
            return False
        # Do not overwrite an upload/IK target with an immediate heartbeat in
        # a depth-one queue. Planned motion needs only a 20 Hz lease renewal.
        if time.monotonic() - self.last_motion_sent_at < .05:
            return True
        if self.motion_request is not None and self.status.motion_id != self.motion_id:
            # Retransmit an idempotent upload until status acknowledges its id.
            # Depth-one ROS queues may replace packets before the server sees them.
            request = self.motion_request.copy()
            goal = request.pop('positions')
            return self.send(goal, **request)
        return self.send((), motion_kind=JointGoal.HEARTBEAT, motion_id=self.motion_id)

    @property
    def motion_complete(self):
        return (self.status is not None and self.status.motion_id == self.motion_id
                and self.status.motion_complete)

    def send(self, goal, velocities=(), **motion):
        if not self.available():
            return False
        positions = ([] if motion.get('motion_kind') == JointGoal.HEARTBEAT
                     else self.spec.validate(goal).tolist())
        velocities = list(velocities)
        if velocities and (len(velocities) != len(self.spec.joints)
                           or not all(math.isfinite(v) for v in velocities)):
            raise ValueError('Feed-forward velocities must contain one finite value per joint')
        self.sent_sequence = max(self.sent_sequence, self.status.sequence) + 1
        self.goals.publish(JointGoal(instance=self.status.instance, session=self.session,
                                    sequence=self.sent_sequence,
                                    action_spec_id=self.spec.identity, positions=positions,
                                    velocities=velocities, **motion))
        self.last_motion_sent_at = time.monotonic()
        return True
