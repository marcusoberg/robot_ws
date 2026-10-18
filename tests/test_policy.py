import asyncio
from dataclasses import replace
import socket
import threading
import time
import numpy as np
import pytest
from robot_description.spec import so101
from model_client.policy import PolicyRunner
from robot_server_core.simulation import Observation
from robot_inference.contracts import Features,Prediction,validate_prediction
from robot_inference.client import RemoteModel
from robot_inference.server import run

class Model:
    def __init__(self):
        self.features=Features(so101().spec.to_json());self.revision='test-revision';self.context=None;self.delay=0.;self.invalid=False
    def describe(self):return self.features
    def predict(self,source):
        time.sleep(self.delay)
        q=list(so101().initial)
        if self.invalid:q[0]=100
        return Prediction(source.basis,source.execution_time_ns,self.revision,self.features.action_representation,.05,[q]*20,10)

def observation(index=0,episode=1):
    return Observation('instance',episode,index,index*50_000_000,time.monotonic_ns(),np.array(so101().initial),np.zeros(6),
                       {k:np.zeros((480,640,3),dtype=np.uint8) for k in so101().spec.cameras})

def finish(runner):
    if runner.pending:runner.pending[0].result(timeout=5)

def test_late_replies_keep_original_sample_times():
    model=Model();runner=PolicyRunner(so101().spec,model)
    try:
        runner.prime(observation());assert runner.action(0).action_index==0
        model.delay=.03
        assert runner.action(5,observation(5)).action_index==5
        # A reply for interval 5 is consumed at interval 10; its first five actions have elapsed.
        finish(runner)
        action=runner.action(10)
        assert action.observation.observation_sequence==5
        assert action.action_index==10 and action.time_ns==500_000_000
        assert runner.action(25) is None
    finally:runner.close()

def test_invalid_reply_does_not_extend_buffer():
    model=Model();runner=PolicyRunner(so101().spec,model)
    try:
        runner.prime(observation());model.invalid=True
        runner.action(5,observation(5));finish(runner)
        assert runner.action(6).observation.observation_sequence==0
        assert runner.action(20) is None
        assert 'prediction_rejected' in runner.diagnostics[-1]
    finally:runner.close()

def test_reset_invalidates_outstanding_predictions():
    model=Model();model.delay=.03;runner=PolicyRunner(so101().spec,model)
    try:
        runner.prime(observation());runner.action(5,observation(5));runner.reset();finish(runner)
        assert runner.action(6) is None and not runner.buffer
    finally:runner.close()

def test_local_and_websocket_predictions_equal():
    with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    model=Model();loop=asyncio.new_event_loop();started=threading.Event();holder={}
    def server():
        asyncio.set_event_loop(loop)
        task=loop.create_task(run(model,'127.0.0.1',port));holder['task']=task;started.set()
        try:loop.run_until_complete(task)
        except asyncio.CancelledError:pass
        finally:loop.run_until_complete(loop.shutdown_asyncgens());loop.close()
    worker=threading.Thread(target=server,daemon=True);worker.start();started.wait()
    remote=None
    try:
        deadline=time.monotonic()+5
        while remote is None:
            try:remote=RemoteModel(f'ws://127.0.0.1:{port}',model.features,expected_revision=model.revision,timeout=1)
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.02)
        runner=PolicyRunner(so101().spec,model)
        try:
            value=runner._input(observation());a=model.predict(value);b=remote.predict(value)
            assert a==b
        finally:runner.close()
    finally:
        if remote:remote.close()
        loop.call_soon_threadsafe(holder['task'].cancel);worker.join(timeout=5)

def test_disconnect_keeps_only_already_buffered_samples():
    model=Model();runner=PolicyRunner(so101().spec,model)
    try:
        runner.prime(observation())
        def disconnected(source):raise ConnectionError('worker disconnected')
        model.predict=disconnected
        runner.action(5,observation(5))
        with pytest.raises(ConnectionError):finish(runner)
        assert runner.action(6).observation.observation_sequence==0
        assert runner.action(20) is None
    finally:runner.close()

def test_policy_cannot_command_or_stop_replacement_source():
    from types import SimpleNamespace
    from robot_server_core.simulation import SimulationEnv
    from model_client.evaluation import run_episode
    env=SimulationEnv(cameras=False)
    class SwitchingRunner:
        reset_called=False
        def prime(self,observation):pass
        def action(self,index,observation):
            if index==1:
                env.pause('source_switch')
                env.activate('replacement')
            return SimpleNamespace(goal=env.profile.initial,observation=SimpleNamespace(observation_sequence=index),time_ns=index*50_000_000)
        def reset(self):self.reset_called=True
    runner=SwitchingRunner()
    try:
        report=run_episode(env,runner,max_seconds=1)
        status=env.native.observe().status
        assert report['reason'].startswith('execution_rejected:')
        assert status.source=='replacement' and status.session and status.mode=='Ready'
        assert status.sequence==0 and runner.reset_called
    finally:env.close()
