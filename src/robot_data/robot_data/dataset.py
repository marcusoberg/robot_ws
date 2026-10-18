from pathlib import Path
import json
import random
import numpy as np
from robot_description.spec import ActionSpec

def convert(recordings, output, repo_id, *, smoke=False):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    output = Path(output)
    if output.exists():
        raise ValueError("Dataset destination already exists")
    episodes = []
    spec = None
    for path in sorted((Path(recordings)/"complete").glob("*/episode.json")):
        meta = json.loads(path.read_text())
        if meta.get("format") != "robot-episode.v2" or meta.get("status") != "complete":
            continue
        if not smoke and (not meta["success"] or meta["purpose"] != "demonstrations"):
            continue
        current = ActionSpec.load(path.parent/"action_spec.json")
        if meta["action_spec"] != current.identity:
            raise ValueError("Episode action specification identity differs")
        if spec:
            spec.require(current)
        spec = current
        episodes.append((path.parent,meta))
    if len(episodes) < 2:
        raise ValueError("At least two complete episodes are needed for an episode-level split")
    identities = [tuple(meta["identity"]) for _,meta in episodes]
    if len(set(identities)) != len(identities):
        raise ValueError("Multiple recordings share a simulation episode; reset between complete demonstrations")
    if 1_000_000_000 % spec.period_ns:
        raise ValueError("LeRobot export requires an integral action frame rate")
    n = len(spec.joints)
    features = {"observation.state": {"dtype":"float32","shape":(n,),"names":list(spec.joints)},
                "action": {"dtype":"float32","shape":(n,),"names":list(spec.joints)},
                "acquisition_time_ns": {"dtype":"int64","shape":(1,),"names":["time"]},
                "application_time_ns": {"dtype":"int64","shape":(1,),"names":["time"]},
                "snapshot_id": {"dtype":"int64","shape":(1,),"names":["id"]},
                **{f"observation.images.camera_{k}": {"dtype":"image","shape":spec.image_shape,"names":["height","width","channels"]} for k in spec.cameras}}
    ds = LeRobotDataset.create(repo_id, fps=round(1e9/spec.period_ns), root=output, features=features,
                               robot_type="so101-position-hold-v2", use_videos=False)
    try:
        for directory, meta in episodes:
            files = sorted(directory.glob("frame-*.npz"))
            if len(files) != meta["frames"]:
                raise ValueError("Episode recording is incomplete")
            previous = None
            for index,path in enumerate(files):
                info = json.loads(path.with_suffix(".json").read_text())
                with np.load(path, allow_pickle=False) as frame:
                    q = spec.validate(frame["action"])
                    if (info["index"] != index or info["action_spec"] != spec.identity
                        or info["acquired_ns"] != info["applied_ns"]
                        or previous is not None and info["applied_ns"] != previous+spec.period_ns
                        or not info["trace"] or any(not np.array_equal(t["reference"],q) for t in info["trace"])):
                        raise ValueError("Episode label or interval provenance is invalid")
                    previous = info["applied_ns"]
                    record = {k:frame[k].astype(np.float32) if k in ("action","observation.state") else frame[k] for k in features if k in frame}
                    record.update(acquisition_time_ns=np.array([info["acquired_ns"]],dtype=np.int64),
                                  application_time_ns=np.array([info["applied_ns"]],dtype=np.int64),
                                  snapshot_id=np.array([info["snapshot"]],dtype=np.int64), task="place the cube inside the container")
                    ds.add_frame(record)
            ds.save_episode()
    finally:
        ds.finalize()
    (output/"meta/action_spec.json").write_text(spec.to_json())
    order = list(range(len(episodes)))
    random.Random(1000).shuffle(order)
    n_val = max(1, int(np.ceil(len(order)*.2)))
    manifest = dict(format="robot-dataset.v2", purpose="smoke" if smoke else "demonstrations", action_spec=spec.identity,
                    repo_id=repo_id, episodes=[{"source":str(d),"identity":m["identity"],"frames":m["frames"]} for d,m in episodes],
                    train=sorted(order[n_val:]), validation=sorted(order[:n_val]))
    (output/"meta/contract.json").write_text(json.dumps(manifest,indent=2))
    return manifest

def validate_dataset(root, spec=None):
    root = Path(root)
    contract = json.loads((root/"meta/contract.json").read_text())
    actual = ActionSpec.load(root/"meta/action_spec.json")
    info = json.loads((root/"meta/info.json").read_text())
    if spec:
        spec.require(actual)
    train, val = contract["train"], contract["validation"]
    if (contract["format"] != "robot-dataset.v2" or contract["action_spec"] != actual.identity
        or not train or not val or set(train)&set(val)
        or sorted(train+val) != list(range(len(contract["episodes"])) )):
        raise ValueError("Invalid dataset contract or episode split")
    if info["fps"]*actual.period_ns != 1_000_000_000 or info["total_episodes"] != len(contract["episodes"]):
        raise ValueError("Dataset timing or episode count differs from its contract")
    for key in ("observation.state","action"):
        feature=info["features"][key]
        if tuple(feature["names"])!=actual.joints or feature["shape"]!=[len(actual.joints)] or feature["dtype"]!="float32":
            raise ValueError(f"Dataset feature order, shape, or dtype differs: {key}")
    cameras={k for k in info["features"] if k.startswith("observation.images.")}
    if cameras!={f"observation.images.camera_{k}" for k in actual.cameras}:
        raise ValueError("Dataset camera schema differs")
    for camera in cameras:
        if tuple(info["features"][camera]["shape"])!=actual.image_shape:
            raise ValueError("Dataset camera resolution differs")
    return actual, contract
