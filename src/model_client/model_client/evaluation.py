import json
from pathlib import Path
import threading
import time
import numpy as np
from robot_server_core.simulation import SimulationEnv
from .policy import PolicyRunner
from robot_server_core.task import PickPlaceSuccess

from robot_server_core.observation import ObservationWorker

def run_episode(env, runner, *, paced=False, max_seconds=60):
    observation=env.reset();success=PickPlaceSuccess();runner.prime(observation)
    session=env.activate('policy');reason='time_limit';steps=0;worker=None;origins=[];max_lag=0
    try:
        if paced:
            initial=runner.action(0)
            if initial is None:raise ValueError('Missing primed action')
            env.native.schedule(initial.goal,session,1,0,True)
            origins.append(dict(index=0,observation=initial.observation.observation_sequence,time_ns=initial.time_ns))
            worker=ObservationWorker(env);env.native.start_paced();index=1
            while True:
                snap=env.native.observe();steps=snap.time//env.profile.spec.period_ns
                max_lag=max(max_lag,snap.wall_lag_seconds)
                if success.update(snap):reason='success';break
                if snap.time>=max_seconds*1e9:break
                if snap.status.mode not in ('Ready','Running') or snap.status.session!=session:
                    reason=snap.status.reason or snap.status.mode;break
                if worker.failure:reason='observation_failure: '+worker.failure;break
                if snap.time>(index-1)*env.profile.spec.period_ns:
                    sample=runner.action(index,worker.latest)
                    if sample is not None:
                        env.native.schedule(sample.goal,session,index+1,index*env.profile.spec.period_ns,True)
                        origins.append(dict(index=index,observation=sample.observation.observation_sequence,time_ns=sample.time_ns))
                        index+=1
                time.sleep(.002)
        else:
            for index in range(round(max_seconds*1e9/env.profile.spec.period_ns)):
                sample=runner.action(index,observation)
                if sample is None:reason='policy_actions_exhausted';break
                origins.append(dict(index=index,observation=sample.observation.observation_sequence,time_ns=sample.time_ns))
                observation,snap=env.step(sample.goal,session=session,sequence=index+1);steps=index+1
                if success.update(snap):reason='success';break
    except (ValueError,RuntimeError) as exc:
        reason=f'execution_rejected: {exc}'
    finally:
        status=env.native.observe().status
        if not status.session or status.session==session:
            env.native.pause(reason)
            if paced:
                deadline=time.monotonic()+5.2
                while env.native.observe().status.mode=='Stopping' and time.monotonic()<deadline:time.sleep(.01)
                env.native.stop_paced()
            else:env.pause(reason)
        if worker:worker.close()
        runner.reset()
    return dict(success=success.success,reason=reason,simulated_seconds=steps*env.profile.spec.period_ns/1e9,
                max_execution_lag_seconds=max_lag,actions=origins)

def evaluate(bundle, output, *, url=None, device='cpu', episodes=10, paced=False):
    from robot_inference.bundle import verify,load_local
    from robot_inference.client import RemoteModel
    from robot_inference.contracts import Features
    if episodes<1:raise ValueError('Episode count must be positive')
    output=Path(output)
    if output.exists():raise ValueError('Evaluation output already exists')
    bundle=Path(bundle);manifest=verify(bundle)
    features=Features.from_json((bundle/'features.json').read_text())
    env=SimulationEnv();env.profile.spec.require(features.spec)
    client=RemoteModel(url,features,expected_revision=manifest['revision'],timeout=10) if url else load_local(bundle,device)
    runner=PolicyRunner(env.profile.spec,client)
    results=[]
    try:
        for _ in range(episodes):
            result=run_episode(env,runner,paced=paced);results.append(result)
            print(json.dumps({k:v for k,v in result.items() if k!='actions'}),flush=True)
    finally:env.close();runner.close()
    latency=np.array(runner.latencies_ns,dtype=float)/1e6
    successes=sum(x['success'] for x in results)
    report=dict(format='robot-evaluation.v2',transport='websocket' if url else 'in-process',driver='paced' if paced else 'stepped',
                model=manifest['revision'],purpose=manifest['purpose'],successes=successes,episodes=episodes,
                gate_passed=manifest['purpose']=='demonstrations' and manifest.get('demonstration_episodes',0)>=20 and episodes==10 and successes>=9,
                latency_ms=dict(p50=float(np.percentile(latency,50)),p95=float(np.percentile(latency,95)),max=float(latency.max())),results=results)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2))
    return report
