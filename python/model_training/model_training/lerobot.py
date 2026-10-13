"""LeRobot recipe adapter; cloud and local jobs use the same entry point."""

from pathlib import Path
from .portable import portable_checkpoint, copy_file
import hashlib
import json
import subprocess
import sys
import os


class LeRobotTrainer:
    def run(self, recipe, dataset, output):
        dataset, output = Path(dataset).resolve(), Path(output).resolve()
        from model_api import Features

        recipe = dict(recipe)
        recipe["features"] = Features.model_validate(recipe.get("features")).model_dump(
            mode="json"
        )
        if recipe.get("model") not in {"act", "smolvla"}:
            raise ValueError("Training model must be act or smolvla")
        if type(recipe.get("steps", 100)) is not int or recipe.get("steps", 100) <= 0:
            raise ValueError("Training steps must be a positive integer")
        device = recipe.get("device")
        if device not in {"cpu", "mps", "cuda"}:
            raise ValueError("Training requires an explicit cpu, mps or cuda device")
        import torch

        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")
        if device == "mps" and not torch.backends.mps.is_available():
            raise RuntimeError("MPS was requested but is unavailable")
        if not (dataset / "meta/info.json").is_file():
            raise ValueError("A local exported LeRobot training split is required")
        if output.exists() and any(output.iterdir()):
            raise ValueError(
                "Use a new output directory; existing jobs are never overwritten"
            )
        output.mkdir(parents=True, exist_ok=True)
        original = str(Path(recipe["checkpoint"]).resolve())
        checkpoint = str(
            portable_checkpoint(
                original,
                output,
                resume=bool(recipe.get("resume")),
                device=device,
                dataset=dataset,
                steps=recipe.get("steps", 100),
            )
        )
        recipe = {**recipe, "source_checkpoint": original, "checkpoint": checkpoint}
        if recipe.get("resume"):
            recipe["resume"] = checkpoint
        run = output / "run"
        if recipe["model"] == "smolvla" and not recipe.get("resume"):
            # LeRobot CLI dictionary overrides merge with checkpoint dictionaries.
            # Materialize an exact adapted feature schema instead of retaining unused cameras.
            source = Path(checkpoint)
            adapted = output / "initial-checkpoint"
            adapted.mkdir(exist_ok=True)
            config = json.loads((source / "config.json").read_text())
            features = recipe["features"]
            config["input_features"] = {
                "observation.state": {
                    "type": "STATE",
                    "shape": [len(features["state_joints"])],
                },
                **{
                    name: {"type": "VISUAL", "shape": list(shape)}
                    for name, shape in features["cameras"].items()
                },
            }
            config["output_features"] = {
                "action": {"type": "ACTION", "shape": [len(features["state_joints"])]}
            }
            (adapted / "config.json").write_text(json.dumps(config, indent=2))

            for file in source.iterdir():
                if file.name == "config.json":
                    continue
                target = adapted / file.name
                if file.is_file() and not target.exists():
                    if file.suffix == ".safetensors":
                        copy_file(file, target)
                    else:
                        copy_file(file, target)
            checkpoint = str(adapted)
            recipe = {**recipe, "checkpoint": checkpoint}

        flags = [
            f"--dataset.repo_id=local/{dataset.name}",
            f"--dataset.root={dataset}",
            f"--output_dir={run}",
            f"--steps={recipe.get('steps', 100)}",
            "--batch_size=4",
            "--seed=42",
            "--dataset.video_backend=pyav",
            "--num_workers=0",
            "--persistent_workers=false",
            "--save_freq=100",
            "--log_freq=10",
            "--env_eval_freq=0",
            "--eval_steps=0",
            "--wandb.enable=false",
            "--policy.push_to_hub=false",
            f"--policy.device={recipe.get('device', 'cpu')}",
        ]
        if recipe.get("resume"):
            flags += [
                "--resume=true",
                f"--config_path={Path(recipe['resume']).resolve() / 'train_config.json'}",
            ]
        else:
            flags += [f"--policy.path={checkpoint}"]
            if recipe["model"] == "act":
                flags += ["--policy.pretrained_backbone_weights=null"]
            if recipe["model"] == "smolvla":
                features = recipe["features"]
                inputs = {
                    "observation.state": {
                        "type": "STATE",
                        "shape": [len(features["state_joints"])],
                    },
                    **{
                        k: {"type": "VISUAL", "shape": shape}
                        for k, shape in features["cameras"].items()
                    },
                }
                flags += [
                    "--policy.input_features=" + json.dumps(inputs),
                    "--policy.output_features="
                    + json.dumps(
                        {
                            "action": {
                                "type": "ACTION",
                                "shape": [len(features["state_joints"])],
                            }
                        }
                    ),
                    "--policy.scheduler_warmup_steps=10",
                    "--policy.scheduler_decay_steps=100",
                ]
        env = {**os.environ, "WANDB_MODE": "disabled", "HF_HUB_DISABLE_TELEMETRY": "1"}
        (output / "recipe.json").write_text(json.dumps(recipe, indent=2))
        with (Path(checkpoint) / "model.safetensors").open("rb") as weights:
            before = hashlib.file_digest(weights, "sha256").hexdigest()
        with (output / "training.log").open("w") as log:
            result = subprocess.run(
                [sys.executable, "-m", "lerobot.scripts.lerobot_train", *flags],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        if result.returncode:
            raise RuntimeError(
                f"Training failed; {output / 'training.log'}\n"
                + (output / "training.log").read_text()[-3000:]
            )
        final = run / "checkpoints" / "last" / "pretrained_model"
        if not final.exists():
            candidates = sorted(run.glob("checkpoints/*/pretrained_model"))
            final = candidates[-1]
        with (final / "model.safetensors").open("rb") as weights:
            after = hashlib.file_digest(weights, "sha256").hexdigest()
        if before == after:
            raise RuntimeError("Training did not change model weights")
        state = final.parent / "training_state"
        if not state.exists():
            raise RuntimeError("Training checkpoint omitted optimizer state")
        from safetensors import safe_open
        import torch

        changed_parameters = []
        with (
            safe_open(
                str(Path(checkpoint) / "model.safetensors"), framework="pt"
            ) as initial,
            safe_open(str(final / "model.safetensors"), framework="pt") as trained,
        ):
            for name in sorted(set(initial.keys()) & set(trained.keys())):
                a, b = initial.get_tensor(name), trained.get_tensor(name)
                if a.shape == b.shape and b.is_floating_point():
                    # Casting is not learning. Compare the original parameter in the saved dtype.
                    delta = (a.to(b.dtype).float() - b.float()).abs()
                    if torch.count_nonzero(delta).item():
                        changed_parameters.append(
                            {"name": name, "max_absolute_change": delta.max().item()}
                        )
        if not changed_parameters:
            raise RuntimeError(
                "No parameter changed beyond checkpoint dtype conversion"
            )
        step = json.loads((state / "training_step.json").read_text())["step"]
        report = {
            "recipe": recipe,
            "initial_sha256": before,
            "trained_sha256": after,
            "checkpoint": str(final),
            "optimizer_state": str(state),
            "changed": True,
            "optimizer_step": step,
            "parameter_changes": changed_parameters,
        }
        (output / "result.json").write_text(json.dumps(report, indent=2))
        return report
