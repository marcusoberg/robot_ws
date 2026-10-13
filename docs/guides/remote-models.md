# Train and serve models on a remote GPU

> **Historical bundle examples:** paths under `artifacts/` and the `act-100`/`smolvla-100` bundles below are local-only and are not included in a clone. Start with the [public ACT walkthrough](../../README.md). For remote ACT serving, transfer its fetched `models/act-base/model` and `src/model_client/config/act.features.json` alongside the model code, or fetch them on the remote host. Substitute those paths and the pinned revision in the serving command below. Training examples require separately supplied datasets/checkpoints and use smoke settings.

The remote machine needs the Python model code, model files and (for training) an exported dataset. It does not need ROS, the C++ robot SDK, MuJoCo, robot assets or a copy of the whole workspace. The robot and Model Client stay on your Mac. The same `policy.v1` Model Server runs locally or remotely.

```text
Training: exported demonstrations + starting weights -> training job -> checkpoint
Serving:  camera/state/text -> Model Client -> Model Server -> joint target rows
Motion:   Model Client -> local Robot Server -> native controller -> robot/simulator
```

First try the [prepared local models](../tutorials/04-local-inference.md). Renting a GPU is optional for deployment. The remote steps below are manual instructions, not an already configured or tested instance.

## 1. Choose and connect to an instance yourself

Use a Linux x86_64 NVIDIA GPU instance with SSH enabled and a working driver. The historical GPU runs used an RTX 5090. There is no measured minimum VRAM requirement for this code; do not interpret a successful 100-step smoke as a throughput/memory benchmark. Leave disk room for the CUDA Python dependencies, datasets and complete checkpoints, including optimizer state and per-job starting copies.

Add your SSH public key through Vast's UI, then copy the connection host and SSH port shown for your instance. Prefer its direct SSH connection when available. Vast documents both connection methods and SSH tunneling in its [SSH guide](https://docs.vast.ai/guides/instances/connect/ssh).

On your Mac, set these placeholders from that UI (not from the archived instance):

```sh
VAST_HOST='root@HOST_FROM_VAST'
VAST_PORT='PORT_FROM_VAST'
ssh -p "$VAST_PORT" "$VAST_HOST"
```

On the remote machine, run `nvidia-smi`. A missing GPU/driver must be fixed before training. Install Python/uv if the template does not provide them, following the [uv installation guide](https://docs.astral.sh/uv/getting-started/installation/). `uv sync --python 3.12` below can provision Python 3.12. Use a normal shell in `/workspace/robot-models`; retain long training jobs in the template's tmux session, or start one yourself.

## 2. Transfer only the model application and bundles

Run on your Mac from `ws`, after setting the variables above. Both ends need `rsync` installed.

```sh
ssh -p "$VAST_PORT" "$VAST_HOST" \
  'mkdir -p /workspace/robot-models/python /workspace/robot-models/environments/models /workspace/robot-models/scripts /workspace/robot-models/models /workspace/robot-models/inputs /workspace/robot-models/datasets'

rsync -a --exclude '__pycache__' --exclude '*.egg-info' \
  -e "ssh -p $VAST_PORT" \
  python/model_api python/model_server python/model_training python/recording_tools \
  "$VAST_HOST:/workspace/robot-models/python/"

rsync -a -e "ssh -p $VAST_PORT" \
  environments/models/pyproject.toml environments/models/uv.lock \
  "$VAST_HOST:/workspace/robot-models/environments/models/"

rsync -a -e "ssh -p $VAST_PORT" scripts/models.py scripts/package_model.py \
  "$VAST_HOST:/workspace/robot-models/scripts/"

rsync -a -e "ssh -p $VAST_PORT" models/act-100 models/smolvla-100 \
  "$VAST_HOST:/workspace/robot-models/models/"
```

`recording_tools` is included because the shared optional environment declares its export command. The Model Server and trainer do not import its recording implementation. This transfer excludes archives, `.git`, ROS installations, private attachment files and old environments. You can transfer only one model folder if you only intend to use that model.

On the remote machine:

```sh
cd /workspace/robot-models
uv sync --locked --python 3.12 --project environments/models
environments/models/.venv/bin/python -c \
  'import torch; print(torch.__version__, torch.version.cuda); assert torch.cuda.is_available(), "CUDA unavailable"; print(torch.cuda.get_device_name(0))'
```

This uses the existing dependency lock (LeRobot 0.6.0, Torch 2.10.0); it does not reuse the template's unrelated Python environment. Installation may download several GB. A host driver incompatible with the locked CUDA wheel must be corrected rather than silently changing the model dependencies.

## 3. Serve an existing model remotely

No dataset is required. On the remote machine, from `/workspace/robot-models`:

```sh
environments/models/.venv/bin/python scripts/models.py serve --model act --checkpoint models/act-100/model \
  --features models/act-100/features.json --revision act-final-100 \
  --device cuda --offline --host 127.0.0.1 --port 8000
```

For SmolVLA replace the model, checkpoint, features and revision with `smolvla`, `models/smolvla-100/model`, `models/smolvla-100/features.json`, `smolvla-final-100`. Its bundle includes the offline tokenizer/config cache; full VLM weights are already in `model.safetensors`.

Expect `Policy ready: ws://127.0.0.1:8000 (policy.v1)`. Leave this process running. In another local terminal:

```sh
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
  -p "$VAST_PORT" -L 127.0.0.1:8001:127.0.0.1:8000 "$VAST_HOST"
```

The model port stays bound to remote loopback. Your local endpoint is now `ws://127.0.0.1:8001`; ROS discovery remains local. This is a regular rented-instance process, not Vast's separate serverless product.

Follow the local walkthrough to launch the matching robot preset, but replace its local Model Server with the remote one. Request a prediction first:

```sh
pixi run smoke inference --config src/model_client/config/act.yaml \
  --url ws://127.0.0.1:8001
```

Then launch the ACT client:

```sh
ros2 launch model_client client.launch.py model_url:=ws://127.0.0.1:8001
```

For SmolVLA use its config and the recorded instruction:

```sh
pixi run smoke inference --config src/model_client/config/smolvla.yaml \
  --url ws://127.0.0.1:8001 \
  --instruction "Pick up the green cube and place it in the brown container."

ros2 launch model_client client.launch.py model_url:=ws://127.0.0.1:8001 \
  config:="$(pwd)/src/model_client/config/smolvla.yaml" \
  instruction:="Pick up the green cube and place it in the brown container."
```

Enable and select the model producer explicitly as in the local walkthrough. Inspect `/robot/result` for native admission/rejections. Network and inference latency consume the original prediction horizon; a cloud GPU does not guarantee better closed-loop behavior than local inference.

Shutdown: explicitly stop/disable the local robot, Ctrl-C the Model Client, Ctrl-C the remote Model Server, then close the SSH tunnel and robot launch. A disconnected SSH terminal is not evidence that a tmux job or rented instance stopped.

## 4. Transfer training inputs

On your Mac, from `ws`:

```sh
rsync -a -e "ssh -p $VAST_PORT" \
  artifacts/legacy-20261011/policy_runner/checkpoints/so101-act-state6/ \
  "$VAST_HOST:/workspace/robot-models/inputs/act-base/"

rsync -a -e "ssh -p $VAST_PORT" \
  artifacts/legacy-20261011/robot-experiments/runs/so101-act-final-dataset/train/ \
  "$VAST_HOST:/workspace/robot-models/datasets/act-train/"

rsync -a -e "ssh -p $VAST_PORT" \
  artifacts/legacy-20261011/robot-experiments/runs/so101-smolvla-final-dataset/train/ \
  "$VAST_HOST:/workspace/robot-models/datasets/smolvla-train/"
```

The training splits contain eight episodes each. Their two-episode validation splits remain separate; never merge validation statistics into training. Only training data is needed by the current smoke-training command. For later evaluation, transfer the `validation/` folders separately too. The current trainer disables automatic environment evaluation.

## 5. Run a 100-step GPU training smoke

On the remote machine, from `/workspace/robot-models`:

```sh
export HF_HUB_CACHE="$PWD/models/smolvla-100/huggingface"

environments/models/.venv/bin/python scripts/models.py train --model act --features models/act-100/features.json \
  --checkpoint inputs/act-base --dataset datasets/act-train \
  --output runs/act-100 --steps 100 --device cuda --offline
```

In another remote terminal, watch `tail -f /workspace/robot-models/runs/act-100/training.log`. A successful run writes `recipe.json`, `result.json`, `training.log` and `run/checkpoints/000100/`. `result.json` records weight changes and optimizer step. Inspect these after the command exits; a short run does not establish task success.

For SmolVLA:

```sh
environments/models/.venv/bin/python scripts/models.py train --model smolvla --features models/smolvla-100/features.json \
  --checkpoint models/smolvla-100/model --dataset datasets/smolvla-train \
  --output runs/smolvla-100 --steps 100 --device cuda --offline
```

This starts from our trained 100-step model with a fresh optimizer. It is not a reproduction from the original SmolVLA base, whose weights are missing locally. The per-job copy sets `load_vlm_weights=false`, because the saved policy includes the VLM weights. The original bundle is not modified.

The current trainer deliberately fixes batch size 4, seed 42, zero loader workers, saving every 100 steps and no online evaluation/publishing. These are smoke settings in `python/model_training/model_training/lerobot.py`, not a tuned training recipe. SmolVLA's fresh-run schedule is also configured for this short smoke. Before longer training, expose/tune those settings and evaluate held-out episodes; changing `--steps` alone is not a complete training experiment.

## 6. Resume with optimizer state

Use a complete training checkpoint, not an inference bundle:

```sh
environments/models/.venv/bin/python scripts/models.py train --model act --features models/act-100/features.json \
  --resume runs/act-100/run/checkpoints/000100/pretrained_model \
  --dataset datasets/act-train --output runs/act-resume \
  --additional-steps 100 --device cuda --offline
```

Expect optimizer step 200 in the new output. SmolVLA uses the same pattern with its names/paths. For direct resume of a *preserved* old run, transfer its entire `000100/` directory containing both `pretrained_model/` and `training_state/`, then point `--resume` at that copied `pretrained_model/`.

`--checkpoint` means initialize weights and start a new optimizer. `--resume` means continue optimizer/scheduler/RNG/step state. The wrapper rewrites saved absolute paths in its own starting copy. LeRobot's [version 0.6.0 training documentation](https://huggingface.co/docs/lerobot/v0.6.0/en/il_robots) describes the same complete-checkpoint resume distinction. Cross-device continuation is not proof of exact replay; local MPS training remains unverified and MPS resume must not claim exact RNG replay.

## 7. Extract, retrieve and deploy your result

On the remote machine:

```sh
environments/models/.venv/bin/python scripts/package_model.py \
  --checkpoint runs/act-resume/run/checkpoints/000200/pretrained_model \
  --features models/act-100/features.json --output models/my-act-200 --label my-act-200
```

For SmolVLA use its features/checkpoint and add `--hf-cache models/smolvla-100/huggingface`. The exporter retains processors and offline resources while omitting optimizer state. Keep complete training runs separately for future resume.

On your Mac, copy both the small deployment bundle and the complete run you want to retain:

```sh
rsync -a -e "ssh -p $VAST_PORT" \
  "$VAST_HOST:/workspace/robot-models/models/my-act-200" models/
mkdir -p runs/from-vast
rsync -a -e "ssh -p $VAST_PORT" \
  "$VAST_HOST:/workspace/robot-models/runs/act-resume" runs/from-vast/
```

Verify the retrieved bundle's manifest on your Mac (this only reads/hashes files):

```sh
python - <<'PY'
from pathlib import Path
import hashlib, json
root = Path('models/my-act-200')
manifest = json.loads((root / 'manifest.json').read_text())
for name, expected in manifest['files'].items():
    with (root / name).open('rb') as stream:
        actual = hashlib.file_digest(stream, 'sha256').hexdigest()
    assert actual == expected['sha256'], name
print('Every listed file matches its SHA-256')
PY
```

Now serve `models/my-act-200/model` locally with `--device mps`, or leave that same bundle on the GPU and serve with `--device cuda`. The client contract is unchanged. For a newly trained model, repeat the synthetic and actual-capture checks before enabling/selecting motion.

Check disk space before retrieving large training outputs. Only after your desired copies are verified should you manually stop/destroy an instance through your own Vast workflow. Nothing in these scripts manages instance lifecycle or contacts the old archived instance.
