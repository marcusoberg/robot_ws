# Optional local training and resume

> **Historical smoke procedure:** the preserved datasets and checkpoints below are not included in this repository. These commands require those local artifacts; they are not a complete training recipe or a prerequisite for running public ACT. See the [main README](../../README.md) for a fresh-clone walkthrough.

These steps are for you to run. They have not been qualified on MPS. First install `pixi run setup-models` and finish a local prediction smoke.

ACT can start from the preserved original base:

```sh
pixi run train-model --model act --features src/model_client/config/act.features.json --checkpoint artifacts/legacy-20261011/policy_runner/checkpoints/so101-act-state6 --dataset artifacts/legacy-20261011/robot-experiments/runs/so101-act-final-dataset/train --output runs/my-act-100 --steps 100 --device mps --offline
pixi run train-model --model act --features src/model_client/config/act.features.json --resume runs/my-act-100/run/checkpoints/000100/pretrained_model --dataset artifacts/legacy-20261011/robot-experiments/runs/so101-act-final-dataset/train --output runs/my-act-resume --additional-steps 100 --device mps --offline
```

SmolVLA's original cloud base weights are missing. Use the preserved trained checkpoint for another fine-tuning run:

```sh
pixi run train-model --model smolvla --features src/model_client/config/smolvla.features.json --checkpoint artifacts/legacy-20261011/robot-experiments/runs/smolvla-final-smoke/run/checkpoints/000100/pretrained_model --dataset artifacts/legacy-20261011/robot-experiments/runs/so101-smolvla-final-dataset/train --output runs/my-smolvla-100 --steps 100 --device mps --offline
pixi run train-model --model smolvla --features src/model_client/config/smolvla.features.json --resume runs/my-smolvla-100/run/checkpoints/000100/pretrained_model --dataset artifacts/legacy-20261011/robot-experiments/runs/so101-smolvla-final-dataset/train --output runs/my-smolvla-resume --additional-steps 100 --device mps --offline
```

This does not reproduce training from the unavailable original SmolVLA base. The complete saved model includes its VLM weights; offline config/tokenizer resources are still required.

Inspect `training.log`, `recipe.json`, `result.json`, the final model, and sibling `training_state`. Expect changed parameters beyond dtype conversion and continued optimizer step count. MPS resume does not establish exact RNG replay. Unsupported MPS operations should be reported; an explicit `--device cpu` run is a separate, slower qualification.

Use new output directories. Never train in preserved checkpoint directories or modify historical metadata. Evaluate predictions separately; a 100-step training smoke does not establish task quality.

For CUDA training on a manually selected Vast instance, use [remote training and deployment](../guides/remote-models.md). The [inference bundles](../../models/README.md) can initialize fresh fine-tuning, but optimizer resume still requires a complete checkpoint.
