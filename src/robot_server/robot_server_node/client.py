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
                report = self.get_logger().info if status.command_accepted else self.get_logger().error
                report(f'{operation}: {status.command_result}')
                self.pending_commands.remove((command_id, operation))
        session = status.session if status.source == self.source else 0
        if session != self.session or previous is None or previous.instance != status.instance:
            self.session = session
            self.sent_sequence = status.sequence if session else 0
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

    def send(self, goal):
        if not self.available():
            return False
        positions = self.spec.validate(goal).tolist()
        self.sent_sequence = max(self.sent_sequence, self.status.sequence) + 1
        self.goals.publish(JointGoal(instance=self.status.instance, session=self.session,
                                    sequence=self.sent_sequence,
                                    action_spec_id=self.spec.identity, positions=positions))
        return True
