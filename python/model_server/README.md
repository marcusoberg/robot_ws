# Model Server

This Python library serves predictions over `policy.v1` WebSockets. It loads a selected local checkpoint on an explicit `cpu`, `mps` or `cuda` device. It does not connect to a robot, acquire control, record episodes, discover ROS nodes or manage cloud instances.

The optional LeRobot adapter is pinned to 0.6.0. Heavy model dependencies belong to `environments/models`; the workspace `scripts/models.py` wrapper runs commands there.

`python -m model_server.serving.server --help` shows the serving options. A server requires `--checkpoint`, `--features`, `--revision`, `--model` and `--device`. `--offline` requires locally available tokenizer/config resources; full SmolVLA checkpoint weights are loaded without requesting a second VLM weight download. The reported model revision is the actual checkpoint SHA-256.

`python -m model_server.inspect --checkpoint PATH` reads configuration, processor references, Safetensors extents and the checkpoint hash without importing Torch or loading the model. Add `--predict --features FILE --device mps --offline` only when you intend to run a finite-prediction smoke check.
