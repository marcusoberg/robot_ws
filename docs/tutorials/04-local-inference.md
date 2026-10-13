# Deploy ACT locally

Follow the [main README's ACT walkthrough](../../README.md#2-install-model-dependencies-and-fetch-act).
It includes fetching the public checkpoint, separate terminal instructions,
Foxglove topics, an inference-only check, explicit enable/selection and shutdown.
It works from a fresh clone and does not need archived checkpoints.

The public model downloads to `models/act-base/model`. Use the tracked feature
contract at `src/model_client/config/act.features.json`. The weights and their
normalization processors belong together; keep the fetched files unchanged.

## Other locally preserved models

The optional `models/act-100` and `models/smolvla-100` bundles from earlier workspace
work are not distributed with this repository. Their 100-step runs established
pipeline execution, not task success. In particular, use the public ACT checkpoint
for the main walkthrough rather than substituting the historical ACT fine-tune.

If you separately possess the SmolVLA bundle and its offline tokenizer resources,
its server command is:

```sh
pixi run model-server --model smolvla --checkpoint models/smolvla-100/model \
  --features src/model_client/config/smolvla.features.json --revision smolvla-final-100 \
  --device mps --offline
```

Choose `cpu`, `mps` or `cuda` explicitly for your machine. The matching robot launch
is `ros2 launch robot_bringup so101_smolvla.launch.py`; its client needs
`config:="$(pwd)/src/model_client/config/smolvla.yaml"` and the task instruction
`Pick up the green cube and place it in the brown container.` This profile uses
one 512×512 RGB camera and records depth separately. Follow the same separate
result-monitor, operator and shutdown sequence as the main README.
