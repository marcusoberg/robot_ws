"""Train from a local checkpoint, with explicitly permitted optional downloads."""

import argparse
import json
import os
from pathlib import Path
from . import LeRobotTrainer


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--recipe", type=Path)
    p.add_argument("--checkpoint", type=Path)
    p.add_argument(
        "--resume",
        type=Path,
        help="Local pretrained_model directory with sibling training_state",
    )
    p.add_argument("--model", choices=["act", "smolvla"])
    p.add_argument("--features", type=Path, help="model_api Features JSON")
    p.add_argument("--device", choices=["cpu", "mps", "cuda"], required=True)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--additional-steps", type=int, default=100)
    p.add_argument("--dataset", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--offline", action="store_true")
    p.add_argument(
        "--allow-download",
        action="store_true",
        help="Resolve the recipe's pinned hub_id/revision only when no local checkpoint is supplied",
    )
    a = p.parse_args()
    if a.steps <= 0 or a.additional_steps <= 0:
        p.error("Training steps must be positive")
    if a.resume and a.checkpoint:
        p.error("Choose --checkpoint or --resume")
    recipe = json.loads(a.recipe.read_text()) if a.recipe else {}
    if "recipe" in recipe:
        recipe = recipe["recipe"]
    if a.model:
        recipe["model"] = a.model
    if a.features:
        recipe["features"] = json.loads(a.features.read_text())
    if recipe.get("model") not in {"act", "smolvla"} or not recipe.get("features"):
        p.error("Supply --model and --features, or a recipe containing both")
    if a.offline:
        os.environ.update(
            HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_DATASETS_OFFLINE="1"
        )
    checkpoint = a.resume or a.checkpoint
    if checkpoint is None:
        if not a.allow_download or a.offline:
            p.error(
                "Supply a local --checkpoint or --resume; downloads require --allow-download"
            )
        if not recipe.get("hub_id") or not recipe.get("revision"):
            p.error("Downloads require a pinned hub_id and revision in --recipe")
        from huggingface_hub import snapshot_download

        checkpoint = Path(
            snapshot_download(
                recipe["hub_id"],
                revision=recipe["revision"],
                allow_patterns=["*.json", "*.safetensors", "README.md"],
            )
        )
    checkpoint = checkpoint.resolve()
    recipe.update(
        checkpoint=str(checkpoint),
        resume=str(checkpoint) if a.resume else None,
        device=a.device,
        steps=a.steps,
    )
    if a.resume:
        saved = checkpoint.parent / "training_state/training_step.json"
        if not saved.is_file():
            p.error("Resume checkpoint omitted training step state")
        recipe["steps"] = json.loads(saved.read_text())["step"] + a.additional_steps
        recipe["additional_steps"] = a.additional_steps
    print(json.dumps(LeRobotTrainer().run(recipe, a.dataset, a.output), indent=2))


if __name__ == "__main__":
    main()
