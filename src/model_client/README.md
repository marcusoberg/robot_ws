# Model Client

This ROS node reads captures, maps named joints/cameras into `policy.v1` inputs, calls an independent WebSocket Model Server, and sends raw timestamped joint targets to `/robot/command`. It owns task text and inference context.

`client.py` keeps ROS callbacks and heartbeats live while one worker waits for inference. Predictions from an old runtime/timeline/session are discarded. Only admitted native predecessors are used for replacement. The client does no IK, interpolation, robot configuration or automatic authority acquisition.

Choose `config/act.yaml` or `config/smolvla.yaml` through `launch/client.launch.py`. Device/checkpoint loading belongs to the Model Server. See the local inference tutorial before selecting the `model` producer.
