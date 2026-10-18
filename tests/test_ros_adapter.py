import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import pytest
rclpy=pytest.importorskip('rclpy',reason='ROS is an optional independent environment')
from robot_server.msg import Control, JointGoal, Status, Observation
from robot_server_node.node import Adapter
from robot_server_core.application import Application

@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','73')


@pytest.fixture
def resolve_ros_command(monkeypatch):
    import runpy
    import yaml
    from launch import LaunchContext
    from launch.actions import ExecuteProcess, IncludeLaunchDescription
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions, visit_all_entities_and_collect_futures
    from ament_index_python.packages import get_package_share_directory

    root=Path(__file__).resolve().parents[1]
    def resolve(invocation):
        if invocation[:2]==['python','scripts/teleop_so101.py']:
            monkeypatch.setattr(sys,'argv',invocation[1:])
            invocation=[]
            monkeypatch.setattr(subprocess,'call',lambda command:invocation.extend(command) or 0)
            wrapper=runpy.run_path(str(root/'scripts/teleop_so101.py'))
            assert wrapper['main']()==0
        launch_index=invocation.index('launch')
        package,filename=invocation[launch_index+1:launch_index+3]
        arguments=dict(item.split(':=',1) for item in invocation[launch_index+3:])
        launch_path=Path(get_package_share_directory(package))/'launch'/filename
        context=LaunchContext()
        processes={}

        def capture_process(action,context):
            command=[perform_substitutions(context,item) for item in action.cmd]
            parameters={}
            for index,item in enumerate(command[:-1]):
                if item=='--params-file':
                    parameters.update(yaml.full_load(Path(command[index+1]).read_text())['/**']['ros__parameters'])
            package=perform_substitutions(context,normalize_to_list_of_substitutions(action.node_package))
            processes[package]=(command,parameters)

        # Evaluate the real nested launches; intercept only the final OS process start.
        monkeypatch.setattr(ExecuteProcess,'execute',capture_process)
        launch=IncludeLaunchDescription(AnyLaunchDescriptionSource(str(launch_path)),
                                        launch_arguments=arguments.items())
        visit_all_entities_and_collect_futures(launch,context)
        return processes,context
    return resolve


@pytest.mark.parametrize('options,bridge_port,server_started',[
    ([],None,False),
    (['start_server:=true'],None,True),
    (['start_server:=true','foxglove:=true'],8766,True),
    (['start_server:=true','foxglove:=true','--foxglove-port','9876'],9876,True),
])
def test_teleop_launch_preserves_serial_port(tmp_path,resolve_ros_command,options,bridge_port,server_started):
    serial_port='/dev/cu.explicit-test-leader'
    calibration=str(tmp_path/'leader calibration.json')
    processes,context=resolve_ros_command(['python','scripts/teleop_so101.py','--port',serial_port,
                                          '--calibration',calibration,*options])
    leader_command,_=processes['robot_teleop']
    assert leader_command[leader_command.index('--port')+1]==serial_port
    assert leader_command[leader_command.index('--calibration')+1]==calibration
    assert context.launch_configurations['port']==serial_port
    assert ('robot_server' in processes)==server_started
    if bridge_port is None:
        assert 'foxglove_bridge' not in processes
    else:
        assert processes['foxglove_bridge'][1]['port']==bridge_port


@pytest.mark.parametrize('task,expected_packages',[
    ('teleop',{'robot_teleop'}),
    ('server',{'robot_server','robot_state_publisher'}),
    ('server-ros',{'robot_server','robot_state_publisher'}),
    ('foxglove',{'foxglove_bridge'}),
])
def test_pixi_startup_tasks_are_independent(tmp_path,resolve_ros_command,task,expected_packages):
    import re
    import shlex
    import tomllib
    root=Path(__file__).resolve().parents[1]
    tasks=tomllib.loads((root/'pixi.toml').read_text())['feature']['ros']['tasks']
    invocation=shlex.split(tasks[task])
    if task == 'teleop':
        invocation.extend(['port:=/dev/cu.explicit-test-leader', f'calibration:={tmp_path / "leader.json"}'])
    processes,_=resolve_ros_command(invocation)
    assert set(processes)==expected_packages
    if task=='foxglove':
        parameters=processes['foxglove_bridge'][1]
        assert parameters['address']=='127.0.0.1' and parameters['port']==8766
        assert any(re.fullmatch(pattern,'package://robot_description/so101/assets/base_so101_v2.stl')
                   for pattern in parameters['asset_uri_allowlist'])


@pytest.mark.parametrize('config', ['', '/tmp/teleop config/so101_ik.toml'])
def test_teleop_launch_passes_optional_ik_config(resolve_ros_command, config):
    processes, _ = resolve_ros_command(['python', 'scripts/teleop_so101.py', '--port', '/dev/test', '--ik-config', config])
    command, _ = processes['robot_teleop']
    assert command[command.index('--ik-config') + 1] == config
    assert set(processes) == {'robot_teleop'}


def command(node,operation,source='test'):
    status=node.status()
    message=Control(command_id=uuid.uuid4().hex,instance=status.instance,episode=status.episode,
                    operation=operation,source=source,session=status.session)
    node.control(message)
    return message


def test_topic_actions_sessions_rejection_and_source_loss(tmp_path):
    rclpy.init(args=[])
    app=Application(tmp_path,cameras=False)
    node=Adapter(app=app)
    try:
        activation=command(node,'activate')
        initial=node.status()
        request=JointGoal(instance=initial.instance,session=initial.session,sequence=1,
                          action_spec_id=node.spec.identity,positions=list(app.env.profile.initial))
        node.goal(request)
        assert node.status().simulation_time_ns==50_000_000
        assert node.status().delivered_sequence==1
        node.goal(request)
        assert node.status().simulation_time_ns==50_000_000
        assert node.status().delivered_sequence==1
        node.control(activation)
        assert node.status().session==initial.session
        command(node,'reset')
        assert node.status().episode>initial.episode
        activation.command_id=uuid.uuid4().hex
        node.control(activation)
        assert not node.status().command_accepted and not node.status().session
        node.goal(request)
        assert node.status().simulation_time_ns==0
        command(node,'activate','replacement')
        assert node.status().session!=initial.session
        replacement=node.status()
        node.goal(JointGoal(instance=replacement.instance,session=replacement.session,sequence=1,
                            action_spec_id=node.spec.identity,positions=list(app.env.profile.initial)))
        node.check_source()
        assert node.status().session==0 and node.status().reason=='source_disconnected'
        node.goal(request)
        assert not node.status().session
        assert not any(name in ('/robot/control','/robot/step') for name,_ in node.get_service_names_and_types())
    finally:
        node.destroy_node();app.close();rclpy.try_shutdown()


def test_source_cannot_submit_for_replacement_owner(tmp_path):
    from robot_server_node.client import Source
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init(args=[])
    app=Application(tmp_path,cameras=False);server=Adapter(app=app);source=Source('test_source',server.spec)
    executor=SingleThreadedExecutor();executor.add_node(server);executor.add_node(source)
    def until(check,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not check():executor.spin_once(timeout_sec=.01)
        assert check()
    try:
        until(lambda:source.status is not None and source.commands.get_subscription_count())
        source.control('activate');until(lambda:bool(source.session))
        assert source.send(app.env.profile.initial)
        assert source.status.delivered_sequence==0
        assert source.send(app.env.profile.initial)
        until(lambda:source.status.delivered_sequence==2)
        assert server.status().simulation_time_ns in (50_000_000,100_000_000)
        command(server,'activate','replacement')
        until(lambda:source.session==0)
        assert not source.send(app.env.profile.initial)
        assert server.status().source=='replacement'
    finally:
        executor.shutdown();source.destroy_node();server.destroy_node();app.close();rclpy.try_shutdown()


def test_launch_topics_foxglove_cameras_tf_and_meshes(tmp_path):
    from websockets.sync.client import connect
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState, Image, CameraInfo
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from robot_description.spec import so101
    from robot_server_node.client import Source
    profile=so101()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    log_path=tmp_path/'launch.log'
    command_line=[sys.executable,'-c','from ros2cli.cli import main; main()','launch','robot_server','server.launch.py',
                  'foxglove:=true',f'foxglove_port:={port}',f'recordings:={tmp_path}/recordings']
    log=log_path.open('w')
    process=subprocess.Popen(command_line,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(args=[]);source=Source('test_external_source',profile.spec)
    ws=None
    targets={'/joint_states':JointState,'/robot/urdf':String,'/tf':TFMessage,'/robot/status':Status,
             **{f'/robot/cameras/{name}/rgb':Image for name in profile.spec.cameras},
             **{f'/robot/cameras/{name}/rgb/camera_info':CameraInfo for name in profile.spec.cameras}}
    received={};channels={};subscriptions={};asset=None;mesh=None
    driver_code = """
import rclpy
from robot_description.spec import so101
from robot_server_node.client import Source
rclpy.init(args=[])
node=Source('test_independent_driver',so101().spec)
activated=False

def drive():
    global activated
    if node.status is not None and node.commands.get_subscription_count() and not activated:
        node.control('activate');activated=True
    if node.available():
        goal=list(so101().initial);goal[0]=.1;node.send(goal)
node.create_timer(.05,drive)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
"""
    driver=subprocess.Popen([sys.executable,'-c',driver_code],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+35
        while time.monotonic()<deadline and (set(received)!=set(targets) or asset is None or source.status is None or source.status.delivered_sequence<2):
            assert process.poll() is None and driver.poll() is None,log_path.read_text()
            rclpy.spin_once(source,timeout_sec=.002)
            if ws is None:
                try:ws=connect(f'ws://127.0.0.1:{port}',subprotocols=['foxglove.sdk.v1'],open_timeout=.1,max_size=8_000_000)
                except OSError:continue
            try:message=ws.recv(timeout=.005)
            except TimeoutError:continue
            if isinstance(message,str):
                value=json.loads(message)
                if value['op']=='advertise':
                    for channel in value['channels']:
                        topic=channel['topic']
                        if topic in targets and topic not in channels:
                            channels[topic]=channel['id'];identifier=len(subscriptions)+1;subscriptions[identifier]=topic
                            ws.send(json.dumps({'op':'subscribe','subscriptions':[{'id':identifier,'channelId':channel['id']}]}))
            elif message[0]==1:
                identifier=struct.unpack_from('<I',message,1)[0];topic=subscriptions[identifier]
                value=deserialize_message(message[13:],targets[topic]);received[topic]=value
                if topic=='/robot/urdf' and mesh is None:
                    uri=ET.fromstring(value.data).find('.//mesh').attrib['filename']
                    assert uri.startswith('package://robot_description/so101/assets/')
                    mesh=profile.description.parent/'assets'/uri.rsplit('/',1)[1]
                    ws.send(json.dumps({'op':'fetchAsset','uri':uri,'requestId':1}))
            elif message[0]==4:
                assert message[5]==0,message[10:].decode(errors='replace')
                assert struct.unpack_from('<I',message,6)[0]==0
                asset=message[10:]
        assert set(received)==set(targets),f'Missing {set(targets)-set(received)}\n{log_path.read_text()}'
        assert asset==mesh.read_bytes()
        assert source.status.delivered_sequence>=2,log_path.read_text()
        assert abs(source.status.measured_positions[0])>0
        assert not any(name in ('/robot/control','/robot/step') for name,_ in source.get_service_names_and_types())
        for name in profile.spec.cameras:
            image=received[f'/robot/cameras/{name}/rgb']
            assert (image.width,image.height,image.encoding)==(640,480,'rgb8')
            assert len(image.data)==640*480*3 and len(set(image.data))>20
        logs=log_path.read_text()
        assert '[ERROR]' not in logs and 'incompatible QoS' not in logs and 'Failed to load' not in logs,logs
    finally:
        if ws:ws.close()
        os.killpg(driver.pid,signal.SIGINT)
        try:driver.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
        source.destroy_node();rclpy.try_shutdown()
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()
    assert 'Traceback' not in log_path.read_text(),log_path.read_text()


def test_ros_policy_primes_then_stops_when_chunk_ends(tmp_path,monkeypatch):
    from rclpy.executors import SingleThreadedExecutor
    from robot_inference.contracts import Features,Prediction
    import model_client.node as module
    rclpy.init(args=[])
    app=Application(tmp_path/'recordings');server=Adapter(app=app)
    features=Features(server.spec.to_json())
    (tmp_path/'features.json').write_text(json.dumps({'action_spec_json':server.spec.to_json()}))
    class Client:
        revision='test-model'
        def predict(self,source):
            assert server.status().mode=='Ready' and server.status().simulation_time_ns==0
            return Prediction(source.basis,source.execution_time_ns,self.revision,features.action_representation,
                              features.action_period,[list(app.env.profile.initial)]*2,0)
    client=Client();client.features=features
    monkeypatch.setattr(module,'verify',lambda bundle:{'revision':'test-model'})
    monkeypatch.setattr(module,'load_local',lambda bundle,device:client)
    policy=module.PolicySource(tmp_path)
    executor=SingleThreadedExecutor();executor.add_node(server);executor.add_node(policy)
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
            if policy.index==2 and not server.status().session:break
        assert policy.index==2
        assert server.status().mode=='Ready' and not server.status().session
        assert not policy.requested and not policy.started
    finally:
        executor.shutdown();policy.destroy_node();server.destroy_node();app.close();rclpy.try_shutdown()


@pytest.mark.parametrize('ik_enabled', [False, True])
def test_teleop_node_uses_topics_and_ends_session_on_bad_sample(tmp_path,monkeypatch,ik_enabled):
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool
    import robot_teleop.so101 as module
    from robot_description.spec import so101
    rclpy.init(args=[])
    app=Application(tmp_path,cameras=False);server=Adapter(app=app)
    class Leader:
        failed=False
        missing=False
        closed=False
        def __init__(self,port,calibration,spec,*,map_positions=True):
            assert port=='/explicit-test-port'
            assert map_positions is not ik_enabled
            self.calibration = {name: dict(id=i+1, drive_mode=0, homing_offset=0, range_min=100, range_max=3900)
                                for i, name in enumerate(spec.joints)}
        def read(self):
            if self.missing:raise module.LeaderFeedbackTimeout('wrist_flex: COMM_RX_TIMEOUT')
            if self.failed:raise ValueError('sample outside calibrated domain')
            return so101().spec.validate(so101().initial),time.monotonic_ns()
        def read_raw(self):
            if self.missing:raise module.LeaderFeedbackTimeout('wrist_flex: COMM_RX_TIMEOUT')
            if self.failed:raise ValueError('sample outside calibrated domain')
            raw = dict.fromkeys(self.calibration,2000)
            raw['gripper'] = self.calibration['gripper']['range_min']
            return raw,time.monotonic_ns()
        def close(self):self.closed=True
    monkeypatch.setattr(module,'SerialLeader',Leader)
    config = Path(__file__).resolve().parents[1] / 'src/robot_teleop/config/so101_ik.toml' if ik_enabled else None
    leader=module.LeaderSource('/explicit-test-port',tmp_path/'calibration.json',keyboard=False,ik_config=config)
    executor=SingleThreadedExecutor();executor.add_node(server);executor.add_node(leader)
    def until(check):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not check():executor.spin_once(timeout_sec=.01)
        assert check()
    try:
        until(lambda:leader.status is not None and leader.commands.get_subscription_count())
        leader.engage(Bool(data=True))
        until(lambda:server.status().delivered_sequence>=2)
        sequence=leader.sent_sequence
        leader.leader.missing=True
        leader.sample()
        assert leader.sent_sequence==sequence
        leader.leader.missing=False
        leader.sample()
        assert leader.sent_sequence==sequence+1
        session=leader.session
        leader.leader.missing=True
        until(lambda:leader.feedback_lost_session==session)
        sequence=leader.sent_sequence
        leader.leader.missing=False
        leader.sample()
        assert leader.sent_sequence==sequence
        until(lambda:not server.status().session and not leader.session)
        assert server.status().mode=='Ready' and not leader.leader.closed
        leader.sample()
        assert not leader.session
        leader.engage(Bool(data=True))
        until(lambda:server.status().session not in (0,session) and server.status().delivered_sequence>=2)
        leader.leader.failed=True
        with pytest.raises(ValueError, match='sample outside calibrated domain'):
            until(lambda:not server.status().session)
        executor.remove_node(leader)
        leader.destroy_node()
        until(lambda:not server.status().session)
        assert server.status().mode=='Ready'
    finally:
        executor.shutdown()
        if not leader.leader.closed:leader.destroy_node()
        server.destroy_node();app.close();rclpy.try_shutdown()
    assert leader.leader.closed


def test_slow_camera_publication_does_not_block_joint_goals(tmp_path,monkeypatch):
    import threading
    from rclpy.executors import SingleThreadedExecutor
    from robot_server_node.client import Source
    rclpy.init(args=[])
    app=Application(tmp_path);server=Adapter(app=app);source=Source('slow_camera_test',server.spec)
    blocked=threading.Event();release=threading.Event()
    def slow_publish(message):
        blocked.set()
        release.wait(10)
    monkeypatch.setattr(server.visualizer.cameras['front'],'publish',slow_publish)
    activated=False
    def drive():
        nonlocal activated
        if source.status is not None and source.commands.get_subscription_count() and not activated:
            source.control('activate');activated=True
        if source.available():source.send(app.env.profile.initial)
    source.create_timer(.05,drive)
    executor=SingleThreadedExecutor();executor.add_node(server);executor.add_node(source)
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and (not blocked.is_set() or server.status().delivered_sequence<5):executor.spin_once(timeout_sec=.01)
        assert server.status().delivered_sequence>=5
        assert server.status().mode=='Running'
        assert blocked.is_set() and not release.is_set()
    finally:
        for node in (server,source):
            for timer in node.timers:timer.cancel()
        release.set();executor.shutdown()
        source.destroy_node();server.destroy_node();app.close();rclpy.try_shutdown()
