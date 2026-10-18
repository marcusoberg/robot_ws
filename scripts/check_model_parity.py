import argparse
import json
from pathlib import Path
import time
import numpy as np
from robot_inference.bundle import load_local
from robot_inference.client import RemoteModel
from robot_inference.contracts import Identity, PolicyInput

def main():
    p=argparse.ArgumentParser(description='Compare local and WebSocket inference on an identical recorded observation')
    p.add_argument('--bundle',required=True);p.add_argument('--frame',type=Path,required=True);p.add_argument('--url',required=True)
    p.add_argument('--device',choices=('cpu','mps','cuda'),default='cpu');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Output already exists')
    local=load_local(a.bundle,a.device)
    with np.load(a.frame,allow_pickle=False) as frame:
        values={k:frame[k].copy() for k in ('observation.state',*local.features.cameras)}
    source=PolicyInput(Identity('parity-check',1,1),0,values,'place the cube inside the container','parity','parity')
    start=time.monotonic_ns();first=local.predict(source);local_ns=time.monotonic_ns()-start
    with RemoteModel(a.url,local.features,expected_revision=local.revision,timeout=30) as remote:
        second=remote.predict(source);remote_ns=remote.last_latency_ns
    difference=float(np.max(np.abs(np.array(first.actions)-second.actions)))
    same=(first.basis==second.basis and first.execution_time_ns==second.execution_time_ns and first.model_revision==second.model_revision
          and first.action_representation==second.action_representation and first.action_period==second.action_period)
    report=dict(passed=same and difference<=1e-6,max_absolute_action_difference=difference,local_ms=local_ns/1e6,
                websocket_ms=remote_ns/1e6,model=local.revision,steps=len(first.actions))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
