"""Export model files and saved processors. Never load or rewrite weights."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import yaml
from .contracts import Features
from .normalization import normalization_report

def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()

def revision(files):
    return "sha256:" + hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()

def verify(root):
    root = Path(root).resolve()
    manifest = json.loads((root / "manifest.json").read_text())
    if manifest["format"] != "robot-inference-bundle.v1":
        raise ValueError("Export this checkpoint with the current model exporter first")
    for name, expected in manifest["files"].items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or digest(path) != expected:
            raise ValueError(f"Bundle resource missing or changed: {name}")
    if revision(manifest["files"]) != manifest["revision"]:
        raise ValueError("Bundle manifest identity mismatch")
    return manifest

def export(checkpoint, deployment, output, hf_cache=None):
    checkpoint, output = Path(checkpoint).resolve(), Path(output).resolve()
    if (checkpoint / "pretrained_model").is_dir():
        checkpoint /= "pretrained_model"
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    config = json.loads((checkpoint / "config.json").read_text())
    profile = yaml.safe_load(Path(deployment).read_text())
    # Mappings are explicit deployment decisions, independent of saved normalization.
    for key in ("state_units", "action_units", "state_scale", "state_offset", "action_scale", "action_offset"):
        if key not in profile:
            raise ValueError(f"Deployment must explicitly define {key}")
    joints = profile["joints"]
    action_joints = profile.get("action_joints", joints)
    for prefix, names in (("state", joints), ("action", action_joints)):
        for suffix in ("units", "scale", "offset"):
            if len(profile[f"{prefix}_{suffix}"]) != len(names):
                raise ValueError(f"{prefix}_{suffix} length differs from joint names")
        for suffix in ("scale", "offset"):
            values = profile[f"{prefix}_{suffix}"]
            if any(not isinstance(v, (int, float)) or not math.isfinite(v) or (suffix == "scale" and v == 0) for v in values):
                raise ValueError(f"Invalid {prefix}_{suffix}")
    train_config = checkpoint / "train_config.json"
    if train_config.is_file():
        dataset_root = json.loads(train_config.read_text()).get("dataset", {}).get("root")
        if dataset_root:
            info = Path(dataset_root) / "meta/info.json"
            if info.is_file() and not math.isclose(profile["period"], 1 / json.loads(info.read_text())["fps"], rel_tol=1e-8):
                raise ValueError("Deployment action period differs from training dataset FPS")
    inputs = config["input_features"]
    cameras = {k: v["shape"] for k, v in inputs.items() if k != "observation.state"}
    if set(cameras) != set(profile["cameras"]) or inputs["observation.state"]["shape"] != [len(joints)]:
        raise ValueError("Deployment inputs differ from checkpoint features")
    if config["output_features"]["action"]["shape"] != [len(action_joints)]:
        raise ValueError("Deployment actions differ from checkpoint features")
    if profile["model"] != config["type"]:
        raise ValueError("Deployment model type differs from checkpoint")
    features = Features(tuple(joints), cameras, profile["period"], config["type"] == "smolvla",
        profile.get("action_representation", "joint_position"), tuple(action_joints),
        tuple(profile["state_units"]), tuple(profile["action_units"]),
        profile.get("binary_gripper", False), profile.get("action_time_offset_ns", 0))
    if features.binary_gripper:
        if profile["action_scale"][-1] != 1 or profile["action_offset"][-1] != 0:
            raise ValueError("Binary gripper output must retain its recorded 0=closed, 1=open convention")
        threshold = profile.get("gripper_threshold", 0.5)
        if not isinstance(threshold, (int,float)) or not 0 < threshold < 1:
            raise ValueError("gripper_threshold must be between zero and one")
    coverage = normalization_report(checkpoint)
    if features.action_representation == "joint_position" and any(not x["statistics_bound"] for x in coverage.values()):
        raise ValueError("Saved processors lack state/action normalization; cannot export robot commands")
    names = {"config.json", "model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"}
    resources = set()
    if config["type"] == "smolvla":
        resources.add(config["vlm_model_name"])
    for pipeline in ("policy_preprocessor.json", "policy_postprocessor.json"):
        for step in json.loads((checkpoint / pipeline).read_text())["steps"]:
            if step.get("state_file"):
                names.add(step["state_file"])
            if step.get("config", {}).get("tokenizer_name"):
                resources.add(step["config"]["tokenizer_name"])
    for name in names:
        path = (checkpoint / name).resolve()
        if not path.is_relative_to(checkpoint) or not path.is_file():
            raise ValueError(f"Missing or external checkpoint resource: {name}")
    caches = []
    for resource in resources:
        if not hf_cache or Path(resource).is_absolute():
            raise ValueError(f"Export requires a Hub identifier and --hf-cache for offline resource: {resource}")
        cache = Path(hf_cache) / ("models--" + resource.replace("/", "--"))
        if not cache.is_dir():
            raise ValueError(f"Missing cached model/tokenizer resource: {cache}")
        for path in cache.rglob("*"):
            if path.is_symlink() and (not path.exists() or not path.resolve().is_relative_to(cache.resolve())):
                raise ValueError(f"Broken or external resource link: {path}")
        caches.append(cache)
    output.parent.mkdir(parents=True, exist_ok=True)
    required = sum((checkpoint / n).stat().st_size for n in names)
    required += sum(p.stat().st_size for c in caches for p in c.rglob("*") if p.is_file())
    if shutil.disk_usage(output.parent).free < required + 64*1024*1024:
        raise ValueError("Insufficient space for model export")
    temporary = Path(tempfile.mkdtemp(prefix=".export-", dir=output.parent))
    try:
        for name in names:
            target = temporary / "model" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(checkpoint / name, target)
        for cache in caches:
            shutil.copytree(cache, temporary / "huggingface" / cache.name, symlinks=False)
        profile["camera_shapes"] = cameras
        profile.pop("expected_revision", None)
        (temporary / "deployment.yaml").write_text(yaml.safe_dump(profile, sort_keys=False))
        (temporary / "features.json").write_text(json.dumps(asdict(features), indent=2)+"\n")
        files = {str(p.relative_to(temporary)): digest(p) for p in sorted(temporary.rglob("*")) if p.is_file()}
        manifest = dict(format="robot-inference-bundle.v1", model_type=config["type"],
                        source=str(checkpoint), revision=revision(files), files=files)
        (temporary / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
        verify(temporary)
        temporary.rename(output)
    except BaseException:
        shutil.rmtree(temporary)
        raise
    print(json.dumps({"bundle": str(output), "revision": manifest["revision"]}, indent=2))

def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("checkpoint", "deployment", "output"):
        p.add_argument("--"+key, type=Path, required=True)
    p.add_argument("--hf-cache", type=Path)
    export(**vars(p.parse_args()))

if __name__ == "__main__":
    main()
