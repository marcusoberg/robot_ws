# Run the original SmolVLA base model locally

This walkthrough uses `lerobot/smolvla_base`, revision
`5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6`, downloaded into
`models/smolvla-base/model`. It leaves upstream weights, config and processor files
unchanged. No training or local fine-tuned model is used.

**Currently this is a diagnostic inference walkthrough, not robot control.**
The original release has no normalization statistics for `observation.state`,
and its action statistics are stored under three legacy SO100 names rather than
`action`. LeRobot 0.6.0 consequently skips both transformations. We preserve that
upstream behavior and label predictions `unmapped_model_output`; we do not claim
the numbers are joint angles. The state supplied to this diagnostic is the
simulator's measured joint positions in radians, unchanged. This establishes
tensor/transport execution only, not correct pretrained state conditioning.

See the [checkpoint findings](../development/smolvla-base.md) for evidence and
what is missing before robot control. The Model Client refuses this profile
before registering a producer. There is no enable/select step in this walkthrough.

## Before starting

Stop any previous robot/model session. If motion is enabled, run these in its
operator terminal before closing processes:

```sh
pixi run robotctl stop --timeout 30
pixi run robotctl disable --timeout 30
```

Then Ctrl-C the old client, model server and robot launch. In each new terminal:

```sh
cd ~/PROJECTS/KTHROBOTICS/ws
pixi shell
```

The model dependencies and base checkpoint are already installed on this machine.
For another machine, install with `pixi run setup-models`, then download the
original model (internet required):

```sh
environments/models/.venv/bin/hf download lerobot/smolvla_base \
  --revision 5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6 \
  --local-dir models/smolvla-base/model
```

Offline serving also needs the pretrained SmolVLM tokenizer/config cache. This
machine already has it under `artifacts/legacy-20261011/huggingface`, selected by
the model launcher. These are upstream tokenizer/config resources, not locally
trained weights. On another machine, set
`HF_HUB_CACHE="$PWD/models/smolvla-base/huggingface"` in Terminal B and start
without `--offline` once to fetch those resources. Subsequent offline starts
discover that bundled cache; keep the base checkpoint path unchanged.

## Terminal A: simulation and Foxglove Bridge

```sh
ros2 launch robot_bringup so101_smolvla_base.launch.py
```

This reuses the existing three-camera SO101 robot configuration. It does not
alter the robot model, scene, joint limits, camera poses or renderer resolution.
It starts disabled and unselected. Keep your Foxglove connection to
`ws://127.0.0.1:8766`; inspect the front, top and wrist RGB topics under
`/robot/cameras/`.

The inference input maps top to `camera1`, wrist to `camera2` and front to
`camera3`. It copies each 640×480 image to 256×192 and pads the top with 64 black
rows to meet the checkpoint's 256×256 input declaration. The original capture
and calibration metadata stay intact. LeRobot subsequently applies the base
model's own image processing.

## Terminal B: original base Model Server

```sh
pixi run model-server --model smolvla \
  --checkpoint models/smolvla-base/model \
  --features src/model_client/config/smolvla_base.features.json \
  --revision 5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6 \
  --device mps --offline
```

Wait for `Policy ready: ws://127.0.0.1:8000 (policy.v1)`. The server also prints
`DIAGNOSTIC ONLY` and its missing normalization bindings. Keep it running.
Use `cpu` instead of `mps` if needed; do not activate another environment.

The reported protocol revision is the actual weights hash, not the readable
`--revision` argument. The inference profile already pins the expected hash.

## Terminal C: one inference request

```sh
pixi run smoke inference \
  --config src/model_client/config/smolvla_base.yaml \
  --instruction "Pick up the green cube and place it in the brown container." \
  --timeout 120
```

Expected fields if inference succeeds:

```json
{
  "robot_control_compatible": false,
  "finite": true,
  "motion_sent": false
}
```

The report includes the original observation identity, camera preparation,
50 predicted six-value vectors, timing, checkpoint hash and the explicit
`unmapped_model_output` representation. It is saved under
`runs/manual-inference-*/report.json`. Repeat this command for another capture.
Finite values do not demonstrate valid joint commands or task success.

The robot will remain stationary. Stop terminals B and A with Ctrl-C when done.
No training, Model Client or authority acquisition is involved.
