import json
from pathlib import Path
import time
import numpy as np
import pytest
from robot_description.spec import so101, ActionSpec
from robot_teleop.leader import LeaderMapping
from robot_server_core.simulation import SimulationEnv
from robot_data.recording import Recorder

@pytest.fixture
def calibration():
    return {j:dict(id=i+1,drive_mode=0,homing_offset=0,range_min=100,range_max=3900) for i,j in enumerate(so101().spec.joints)}

def test_full_leader_travel(calibration):
    spec=so101().spec;m=LeaderMapping(spec,calibration)
    for i,joint in enumerate(spec.joints):
        previous=-float('inf')
        for value in np.linspace(100,3900,1001):
            sample=dict.fromkeys(spec.joints,2000);sample[joint]=value
            q=m.map(sample);spec.validate(q);assert q[i]>=previous;previous=q[i]
    assert np.array_equal(m.map(dict.fromkeys(spec.joints,100)),spec.lower)
    assert np.array_equal(m.map(dict.fromkeys(spec.joints,3900)),spec.upper)
    assert np.array_equal(m.map(dict.fromkeys(spec.joints,2000))[:5],np.zeros(5))
    for value in (99,3901,float('nan'),float('inf')):
        with pytest.raises(ValueError):m.map(dict.fromkeys(spec.joints,value))
    calibration[spec.joints[0]]['range_max']=100
    with pytest.raises(ValueError):LeaderMapping(spec,calibration)

def test_stepping_deterministic_and_expiry():
    env=SimulationEnv(cameras=False)
    try:
        arrays=[]
        for _ in range(2):
            env.reset();env.activate('test');states=[]
            for i in range(10):
                if _==0 and i==5:time.sleep(.35)
                q=list(env.profile.initial);q[0]=i*.005
                obs,snap=env.step(q)
                assert snap.time==(i+1)*50_000_000
                assert len(snap.trace)==5 and all(t.reference==q for t in snap.trace)
                states.append(obs.position)
            arrays.append(states)
        assert np.array_equal(arrays[0],arrays[1])
        session=env.session;before=env.native.observe()
        with pytest.raises(ValueError):env.step([float('nan')]*6)
        assert env.native.observe().time==before.time
        env.native.advance(30)
        assert env.native.observe().status.mode in ('Stopping','Ready')
        with pytest.raises(ValueError):env.native.submit(env.profile.initial,session,100)
    finally:env.close()

def test_native_runs_without_python_updates():
    env=SimulationEnv(cameras=False)
    try:
        env.activate('test');env.native.submit(env.profile.initial,env.session,1)
        env.native.start_paced();time.sleep(.45)
        snap=env.native.observe()
        assert snap.time>=300_000_000
        assert snap.status.mode in ('Stopping','Ready')
        assert snap.status.session==0
    finally:env.close()

def test_camera_record_alignment(tmp_path):
    env=SimulationEnv()
    recorder=Recorder(tmp_path,env.profile.spec)
    try:
        before=env.reset();env.activate('test');recorder.start(before)
        goal=env.profile.initial;after,snap=env.step(goal)
        recorder.append(before,goal,after,snap)
        path=Path(recorder.finish(complete=True))
        with np.load(path/'frame-000000.npz') as frame:
            assert np.array_equal(frame['action'],goal)
            assert np.array_equal(frame['observation.state'],before.position)
            assert not np.array_equal(frame['observation.state'],after.position)
            for camera in env.profile.spec.cameras:assert frame[f'observation.images.camera_{camera}'].shape==(480,640,3)
        meta=json.loads((path/'frame-000000.json').read_text())
        assert meta['snapshot']==before.snapshot and meta['acquired_ns']==meta['applied_ns']==0
        recorder.start(env.reset())
        path=Path(recorder.interrupt('source_lost'))
        assert json.loads((path/'episode.json').read_text())['status']=='incomplete'
    finally:recorder.close();env.close()

def test_old_and_wrong_contracts_rejected():
    data=json.loads(so101().spec.to_json());data['version']=1
    with pytest.raises(ValueError):ActionSpec(**data)
    data=json.loads(so101().spec.to_json());data['units'][0]='degree'
    with pytest.raises(ValueError):ActionSpec(**data)
    data=json.loads(so101().spec.to_json());data['joints'].reverse()
    with pytest.raises(ValueError):so101().spec.require(ActionSpec(**data))

def test_bounded_paced_queue_and_pause_clears_pending():
    env=SimulationEnv(cameras=False)
    try:
        env.activate('test');session=env.session
        env.native.schedule(env.profile.initial,session,1,0)
        with pytest.raises(ValueError):env.native.schedule(env.profile.initial,session,2,50_000_000)
        env.pause()
        with pytest.raises(ValueError):env.native.schedule(env.profile.initial,session,3,0)
        env.activate('new-source');env.native.start_paced();time.sleep(.03)
        with pytest.raises(ValueError):env.native.schedule(env.profile.initial,env.session,1,0)
    finally:env.close()

def test_backend_rejects_model_mapping_discrepancy(tmp_path):
    from robot_runtime_native._native import Simulation
    p=so101();spec=p.native_spec();spec.lower=[v-.001 for v in p.spec.lower]
    with pytest.raises(ValueError,match='limits disagree'):
        Simulation(spec,str(p.scene),p.initial)

def test_action_interval_exhaustion_stops_at_boundary():
    env=SimulationEnv(cameras=False)
    try:
        env.activate('policy');q=list(env.profile.initial);q[0]=.1
        env.native.schedule(q,env.session,1,0,True)
        snap=env.native.advance(5)
        assert snap.time==50_000_000 and snap.status.mode=='Running'
        assert all(t.reference==q for t in snap.trace)
        snap=env.native.advance(1)
        assert snap.status.mode=='Stopping' and snap.status.reason=='action_interval_exhausted'
        assert snap.trace[0].begin==50_000_000 and snap.trace[0].sequence==0
    finally:env.close()

def test_task_success_requires_supported_released_dwell():
    from types import SimpleNamespace as N
    from robot_server_core.task import PickPlaceSuccess
    rotation=np.eye(3).reshape(-1).tolist()
    contact=N(geom1='block',geom2='container_bottom',normal_force=.5,ancestors1=['block'],ancestors2=['container'])
    snap=N(time=0,bodies={'block':[.1,.1,.02]+rotation,'container':[.1,.1,0.]+rotation},contacts=[contact])
    success=PickPlaceSuccess();assert not success.update(snap)
    snap.time=999_999_999;assert not success.update(snap)
    snap.time=1_000_000_000;assert success.update(snap)
    for reason in ('no_support','gripped','outside'):
        success=PickPlaceSuccess()
        snap.time=0;snap.bodies['block']=[.1,.1,.02]+rotation;snap.contacts=[contact]
        if reason=='no_support':snap.contacts=[]
        if reason=='gripped':snap.contacts.append(N(geom1='block',geom2='',normal_force=.1,ancestors1=['block'],ancestors2=['gripper']))
        if reason=='outside':snap.bodies['block'][0]=.2
        assert not success.update(snap)
        snap.time=2_000_000_000;assert not success.update(snap)

def test_backend_rejects_nonposition_actuator(tmp_path):
    from robot_runtime_native._native import RobotSpec, Simulation
    spec=RobotSpec();spec.joints=['j0','j1','j2'];spec.units=['rad']*3;spec.lower=[-1.]*3;spec.upper=[1.]*3;spec.backend='test-mujoco'
    path=tmp_path/'robot.xml'
    path.write_text('''<mujoco><compiler angle="radian"/><option timestep=".001"><flag clampctrl="disable"/></option>
    <worldbody><body><joint name="j0" axis="1 0 0" range="-1 1"/><joint name="j1" axis="0 1 0" range="-1 1"/>
    <joint name="j2" axis="0 0 1" range="-1 1"/><geom type="sphere" size=".1" mass="1"/></body></worldbody>
    <actuator><motor joint="j0" ctrlrange="-1 1"/><position joint="j1" ctrlrange="-1 1"/><position joint="j2" ctrlrange="-1 1"/></actuator></mujoco>''')
    with pytest.raises(ValueError,match='position actuator'):Simulation(spec,str(path),[0.]*3)

def test_serial_leader_checks_device_calibration_without_clamping(calibration,tmp_path,monkeypatch):
    import serial
    from robot_teleop.leader import SerialLeader
    class SerialWire:
        position=2000;minimum=100;error=0
        connections=[];writes=[]
        def __init__(self,**settings):
            assert settings['port']=='/explicit-port' and settings['baudrate']==1_000_000
            self.closed=False;self.incoming=bytearray();self.connections.append(self)
        def close(self):self.closed=True
        def reset_input_buffer(self):self.incoming.clear()
        def flush(self):pass
        def read(self,length):
            data=bytes(self.incoming[:length]);del self.incoming[:length];return data
        def write(self,packet):
            assert packet[:2]==[255,255] and packet[3]+4==len(packet)
            assert sum(packet[2:])&255==255
            motor,instruction,address=packet[2],packet[4],packet[5]
            if instruction==2:
                assert packet[6]==2
                value={31:2048+123,9:self.minimum,11:3900,56:self.position}[address]
                payload=[value&255,value>>8]
            else:
                assert instruction==3 and address==40 and packet[6]==0
                self.writes.append((motor,address,packet[6]));payload=[]
            response=[255,255,motor,len(payload)+2,self.error,*payload]
            response.append(~sum(response[2:])&255)
            self.incoming.extend(response)
            return len(packet)
    monkeypatch.setattr(serial,'Serial',SerialWire)
    for motor in calibration.values():motor['homing_offset']=-123
    path=tmp_path/'calibration.json';path.write_text(json.dumps(calibration))
    leader=SerialLeader('/explicit-port',path,so101().spec)
    try:
        goal,stamp=leader.read();assert np.array_equal(goal[:5],np.zeros(5)) and stamp>0
        assert SerialWire.writes==[(i,40,0) for i in range(1,7)]
        SerialWire.position=99
        with pytest.raises(ValueError,match='outside calibrated'):leader.read()
        SerialWire.position=2000;SerialWire.error=4
        with pytest.raises(RuntimeError,match='feedback failed'):leader.read()
        SerialWire.error=0
    finally:leader.close()
    assert all(port.closed for port in SerialWire.connections)
    SerialWire.minimum=101
    with pytest.raises(ValueError,match='differs from calibration'):SerialLeader('/explicit-port',path,so101().spec)
    assert all(port.closed for port in SerialWire.connections)
    assert json.loads(path.read_text())==calibration
