# Optional Model Environment

This uv project holds LeRobot 0.6.0, Torch, TorchVision, TorchCodec and the local model/training/export adapters. It is separate from the root Pixi ROS environment so robot deployment and recording do not require model dependencies.

Run model commands through the workspace `scripts/models.py` wrapper. The checked-in `uv.lock` resolves macOS arm64 and Linux x86_64 dependencies. Creating or syncing this environment does not start a model server, train a model or manage a cloud instance.
