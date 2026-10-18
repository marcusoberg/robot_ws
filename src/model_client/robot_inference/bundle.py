import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
from robot_description.spec import ActionSpec
from .contracts import Features
from .normalization import normalization_report

def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream,"sha256").hexdigest()

def revision(files):
    return "sha256:"+hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()

def verify(root):
    root = Path(root).resolve()
    manifest = json.loads((root/"manifest.json").read_text())
    if manifest["format"] != "robot-inference-bundle.v2" or manifest["model_type"] != "act":
        raise ValueError("Model does not declare the v2 position-goal contract")
    required = {"action_spec.json","features.json","model/config.json","model/model.safetensors","model/policy_preprocessor.json","model/policy_postprocessor.json","model/training_contract.json"}
    if not required.issubset(manifest["files"]):
        raise ValueError("Incomplete model bundle")
    for name, expected in manifest["files"].items():
        path = (root/name).resolve()
        if not path.is_relative_to(root) or digest(path) != expected:
            raise ValueError(f"Bundle resource missing or changed: {name}")
    if revision(manifest["files"]) != manifest["revision"]:
        raise ValueError("Bundle manifest identity mismatch")
    spec = ActionSpec.load(root/"action_spec.json")
    spec.require(Features.from_json((root/"features.json").read_text()).spec)
    trained=json.loads((root/"model/training_contract.json").read_text())
    expected_count=len(trained["dataset"]["episodes"]) if trained["dataset"]["purpose"]=="demonstrations" else 0
    if (manifest["action_spec"] != spec.identity or manifest["controller"] != spec.controller
        or manifest["purpose"] != trained["dataset"]["purpose"] or manifest.get("demonstration_episodes",0)!=expected_count):
        raise ValueError("Bundle action contract differs")
    return manifest

def export(checkpoint, output):
    checkpoint,output = Path(checkpoint).resolve(),Path(output).resolve()
    if output.exists():
        raise ValueError("Export destination already exists")
    spec = ActionSpec.load(checkpoint/"action_spec.json")
    trained = json.loads((checkpoint/"training_contract.json").read_text())
    if trained["format"] != "robot-training.v2" or trained["dataset"]["action_spec"] != spec.identity:
        raise ValueError("Checkpoint lacks the required training action semantics")
    config = json.loads((checkpoint/"config.json").read_text())
    features = Features(spec.to_json())
    expected = {"observation.state":[len(spec.joints)],**{k:list(v) for k,v in features.cameras.items()}}
    if (config["type"] != "act" or {k:v["shape"] for k,v in config["input_features"].items()} != expected
        or config["output_features"]["action"]["shape"] != [len(spec.joints)]
        or config.get("temporal_ensemble_coeff") is not None):
        raise ValueError("Checkpoint features or controller semantics differ")
    if any(not item["statistics_bound"] for item in normalization_report(checkpoint).values()):
        raise ValueError("Checkpoint normalization is incomplete")
    names = {"config.json","model.safetensors","policy_preprocessor.json","policy_postprocessor.json","training_contract.json"}
    for pipeline in ("policy_preprocessor.json","policy_postprocessor.json"):
        for step in json.loads((checkpoint/pipeline).read_text())["steps"]:
            if step.get("state_file"):
                names.add(step["state_file"])
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".export-",dir=output.parent))
    try:
        for name in names:
            path=(checkpoint/name).resolve()
            if not path.is_relative_to(checkpoint):
                raise ValueError("External model resource")
            destination=temporary/"model"/name
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(path,destination)
        (temporary/"action_spec.json").write_text(spec.to_json())
        (temporary/"features.json").write_text(json.dumps(asdict(features)))
        files={str(p.relative_to(temporary)):digest(p) for p in sorted(temporary.rglob("*")) if p.is_file()}
        manifest=dict(format="robot-inference-bundle.v2",model_type="act",revision=revision(files),files=files,action_spec=spec.identity,
                      purpose=trained["dataset"]["purpose"],controller=spec.controller,
                      demonstration_episodes=len(trained["dataset"]["episodes"]) if trained["dataset"]["purpose"]=="demonstrations" else 0)
        (temporary/"manifest.json").write_text(json.dumps(manifest,indent=2))
        verify(temporary)
        temporary.rename(output)
        return manifest
    except BaseException:
        shutil.rmtree(temporary)
        raise

def load_local(bundle, device):
    from .lerobot import LeRobotModel
    root=Path(bundle)
    manifest=verify(root)
    features=Features.from_json((root/"features.json").read_text())
    model=LeRobotModel.load(str(root/"model"),features=features,revision=manifest["revision"],device=device,model_type="act")
    model.revision=manifest["revision"]
    return model

def main():
    p=argparse.ArgumentParser(description="Export an ACT checkpoint with its v2 action contract")
    p.add_argument("--checkpoint",required=True);p.add_argument("--output",required=True)
    print(json.dumps(export(**vars(p.parse_args())),indent=2))
if __name__=="__main__":main()
