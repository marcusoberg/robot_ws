# Model Training

This Python library trains from exported local LeRobot datasets and local checkpoints. It has no ROS or robot-control dependency. The optional LeRobot 0.6.0 training dependencies are installed separately in `environments/models`.

`python -m model_training.prepare --help` shows the job arguments. Choose an explicit `--device cpu`, `mps` or `cuda`, a `--model`, a feature JSON, a local `--dataset`, and a new `--output` directory. `--checkpoint` starts a new job; `--resume` loads a saved checkpoint together with its optimizer/RNG/scheduler state. A fresh job defaults to 100 steps; resume defaults to 100 additional steps.

Each job first creates a private checkpoint copy, using a filesystem clone when supported. It rewrites stale saved dataset/output/checkpoint paths and the requested device in that copy. Preserved historical checkpoints and their metadata stay unchanged. Full SmolVLA checkpoints construct their architecture without downloading VLM weights; offline jobs also need the cached tokenizer/config resources.

Use `--offline` for local jobs. Hub retrieval is opt-in through `--allow-download` and a recipe containing a pinned model/revision. The job writes its recipe, training log and a result with saved optimizer step, weight hashes and parameter changes. Previous CUDA qualification does not establish MPS training or exact MPS RNG replay.
