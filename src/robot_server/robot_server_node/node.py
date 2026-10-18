import json
from collections import deque
import rclpy
from rclpy.node import Node
from .visualization import Visualization
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from robot_server_core.observation import ObservationWorker
from rosgraph_msgs.msg import Clock
from robot_server.msg import Status, JointGoal, Control
from std_msgs.msg import Float64MultiArray, String
from robot_server_core.application import Application
from robot_description.spec import load_profile

class Adapter(Node):
    def __init__(self, app=None):
        super().__init__('robot_server',start_parameter_services=False)
        recordings=self.declare_parameter('recordings','recordings').value
        robot=self.declare_parameter('robot','so101').value
        gripper=self.declare_parameter('gripper','pneumatic').value
        self.paced=self.declare_parameter('paced',False).value
        self.owns_application=app is None
        self.app=Application(recordings,profile=load_profile(robot,gripper=gripper)) if self.owns_application else app
        if not self.owns_application and self.paced:
            raise ValueError('An attached adapter cannot own the application clock')
        self.spec=self.app.env.profile.spec
        qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=1)
        self.states=self.create_publisher(JointState,'/joint_states',qos)
        self.statuses=self.create_publisher(Status,'/robot/status',qos)
        self.clock=self.create_publisher(Clock,'/clock',qos)
        self.worker=ObservationWorker(self.app.env) if self.paced else None
        self.command_id='';self.command_result='';self.command_accepted=False
        self.seen_commands=deque(maxlen=128)
        self.references=self.create_publisher(JointState,'/robot/reference',qos)
        self.tracking=self.create_publisher(Float64MultiArray,'/robot/tracking_error',qos)
        self.timing=self.create_publisher(Float64MultiArray,'/robot/timing',qos)
        self.ownership=self.create_publisher(String,'/robot/ownership',qos)
        self.create_subscription(Control,'/robot/control',self.control,10)
        self.create_subscription(JointGoal,'/robot/joint_goal',self.goal,qos)
        self.create_timer(.05,self.publish_status)
        self.create_timer(.05,self.check_source)
        if self.paced:self.app.env.native.start_paced()
        self.visualizer=Visualization(self.app.env,lambda:self.worker.latest if self.worker else self.app.current)
        self.get_logger().info(f'{self.app.env.profile.description.stem} simulation ready: /robot/joint_goal and /robot/control topics; initially paused')

    def status(self):
        snap=self.app.env.native.observe();s=snap.status
        return Status(mode=s.mode,instance=self.app.env.instance,reason=s.reason,source=s.source,session=s.session,sequence=s.sequence,
            delivered_sequence=s.delivered_sequence,episode=snap.episode,simulation_time_ns=snap.time,
            execution_time_ns=s.time,action_spec_id=self.spec.identity,joints=list(self.spec.joints),
            measured_positions=s.measured.position,measured_velocities=s.measured.velocity,commanded_reference=s.reference,
            recording=bool(self.app.recorder.active),command_id=self.command_id,command_result=self.command_result,command_accepted=self.command_accepted)

    def packet(self, value):
        status=self.status()
        if value.instance!=status.instance or value.action_spec_id!=self.spec.identity or not value.session or value.session!=status.session or value.sequence<=status.sequence:
            raise ValueError('Stale session, reordered packet, or incompatible action specification')
        self.spec.validate(value.positions)

    def goal(self, message):
        try:
            self.packet(message)
            if self.paced:
                self.app.env.native.submit(message.positions,message.session,message.sequence,self.app.env.profile.source_timeout_ns)
            else:
                self.app.step(message.positions,session=message.session,sequence=message.sequence)
        except Exception as exc:
            self.get_logger().warning(f'Joint goal rejected: {exc}')
        self.publish_status()

    def check_source(self):
        if self.worker and self.worker.failure:
            self.app.env.native.pause('camera_acquisition_failed')
            return
        status=self.status()
        if not status.session or status.mode!='Running':
            return
        publishers={info.node_namespace.rstrip('/')+'/'+info.node_name
                    for info in self.get_publishers_info_by_topic('/robot/joint_goal')}
        if status.source not in publishers:
            self.app.pause('source_disconnected')
            self.publish_status()

    def control(self, message):
        if not message.command_id or message.command_id in self.seen_commands:
            return
        self.seen_commands.append(message.command_id)
        self.command_id=message.command_id
        self.command_accepted=False
        try:
            status=self.status()
            if message.instance!=status.instance:
                raise ValueError('Control command belongs to an earlier server instance')
            if message.episode!=status.episode or message.session!=status.session:
                raise ValueError('Control command belongs to an earlier episode or source session')
            if message.operation=='activate':
                if not message.source:raise ValueError('Activation requires a source identity')
                result=self.app.activate(message.source)
            else:
                if message.operation=='record' and self.paced:
                    raise ValueError('Recording requires stepped intervals in this release')
                result=self.app.command(message.operation)
                if message.operation=='reset' and self.paced:self.app.env.native.start_paced()
            self.command_result=json.dumps(result)
            self.command_accepted=True
        except Exception as exc:
            self.command_result=str(exc)
            self.get_logger().warning(f'Control {message.operation} rejected: {exc}')
        self.publish_status()

    @staticmethod
    def stamp(time_ns, header):
        header.stamp.sec,header.stamp.nanosec=divmod(time_ns,1_000_000_000)

    def publish_status(self):
        status=self.status();self.statuses.publish(status)
        joint=JointState(name=list(self.spec.joints),position=status.measured_positions,velocity=status.measured_velocities)
        self.stamp(status.simulation_time_ns,joint.header);self.states.publish(joint)
        self.references.publish(JointState(header=joint.header,name=joint.name,position=status.commanded_reference))
        if len(status.commanded_reference)==len(status.measured_positions):
            self.tracking.publish(Float64MultiArray(data=[q-m for q,m in zip(status.commanded_reference,status.measured_positions)]))
        self.timing.publish(Float64MultiArray(data=[status.simulation_time_ns/1e9,status.execution_time_ns/1e9,self.app.env.native.observe().wall_lag_seconds]))
        self.ownership.publish(String(data=json.dumps(dict(mode=status.mode,source=status.source,session=status.session,reason=status.reason))))
        clock=Clock();clock.clock=joint.header.stamp;self.clock.publish(clock)

    def destroy_node(self):
        self.visualizer.destroy_node()
        if self.owns_application:self.app.env.native.disable()
        if self.worker:self.worker.close()
        if self.owns_application:self.app.close()
        super().destroy_node()

def main():
    import signal
    rclpy.init();node=Adapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node();rclpy.try_shutdown()
