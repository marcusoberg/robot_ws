"""Make per-job checkpoint copies before changing stale saved paths."""

from pathlib import Path
import json
import shutil
import subprocess
import sys


def copy_file(source, destination):
    source, destination = Path(source), Path(destination)
    # Clone where supported; fallback copies never share mutable hard links.
    command = (
        ["cp", "-c", str(source), str(destination)]
        if sys.platform == "darwin"
        else ["cp", "--reflink=auto", str(source), str(destination)]
    )
    try:
        subprocess.run(
            command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    except (OSError, subprocess.CalledProcessError):
        shutil.copy2(source, destination)
    return str(destination)


def portable_checkpoint(checkpoint, output, *, resume, device, dataset, steps):
    source = Path(checkpoint).resolve()
    if not source.is_dir() or not (source / "model.safetensors").is_file():
        raise ValueError("A local pretrained_model checkpoint directory is required")
    for name in [
        "config.json",
        "policy_preprocessor.json",
        "policy_postprocessor.json",
    ]:
        if not (source / name).is_file():
            raise ValueError(f"Checkpoint omitted {name}")
    root = Path(output) / "starting-checkpoint"
    target = root / "pretrained_model"
    shutil.copytree(source, target, copy_function=copy_file)
    config = json.loads((target / "config.json").read_text())
    config["device"] = device
    if config.get("type") == "act":
        config["pretrained_backbone_weights"] = None
    if config.get("type") == "smolvla":
        config["load_vlm_weights"] = False
    (target / "config.json").write_text(json.dumps(config, indent=2))
    if resume:
        state = source.parent / "training_state"
        for name in [
            "training_step.json",
            "optimizer_state.safetensors",
            "optimizer_param_groups.json",
            "rng_state.safetensors",
        ]:
            if not (state / name).is_file():
                raise ValueError(f"Resume checkpoint omitted {name}")
        shutil.copytree(state, root / "training_state", copy_function=copy_file)
        path = target / "train_config.json"
        if not path.is_file():
            raise ValueError("Resume checkpoint omitted train_config.json")
        saved = json.loads(path.read_text())
        saved.update(output_dir=str(Path(output) / "run"), steps=steps, resume=True)
        saved["dataset"]["root"] = str(dataset)
        saved["policy"]["device"] = device
        saved["policy"]["pretrained_path"] = str(target)
        if saved["policy"].get("type") == "act":
            saved["policy"]["pretrained_backbone_weights"] = None
        if saved["policy"].get("type") == "smolvla":
            saved["policy"]["load_vlm_weights"] = False
        path.write_text(json.dumps(saved, indent=2))
    return target
