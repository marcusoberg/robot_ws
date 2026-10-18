import json
from pathlib import Path
import numpy as np
from robot_description.spec import ActionSpec
from robot_server_core.simulation import SimulationEnv

def replay(episode, output):
    episode,output=Path(episode),Path(output)
    if output.exists():raise ValueError('Replay report already exists')
    manifest=json.loads((episode/'episode.json').read_text())
    if manifest.get('format')!='robot-episode.v2' or manifest.get('status')!='complete':
        raise ValueError('Replay requires a complete v2 episode')
    spec=ActionSpec.load(episode/'action_spec.json')
    files=sorted(episode.glob('frame-*.npz'))
    if len(files)!=manifest['frames'] or not files:raise ValueError('Incomplete episode')
    env=SimulationEnv();env.profile.spec.require(spec)
    error=0.;labels=True
    try:
        observation=env.reset();env.activate('replay')
        for path in files:
            info=json.loads(path.with_suffix('.json').read_text())
            with np.load(path,allow_pickle=False) as frame:
                if observation.time_ns!=info['acquired_ns']:raise ValueError('Replay requires an episode recorded from reset')
                error=max(error,float(np.max(np.abs(observation.position-frame['observation.state']))))
                observation,snapshot=env.step(frame['action'])
                error=max(error,float(np.max(np.abs(observation.position-frame['subsequent.state']))))
                labels &= all(np.array_equal(t.reference,frame['action']) for t in snapshot.trace)
    finally:env.close()
    result=dict(format='robot-replay.v2',passed=error<1e-10 and labels,max_state_error=error,labels_match_delivery=bool(labels),frames=len(files))
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2))
    if not result['passed']:raise RuntimeError(f'Replay differs from recording: {result}')
    return result
