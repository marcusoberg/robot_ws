# Commands and interfaces

Activate from `ws`. `pixi run doctor` checks installation without starting a robot.

| Command | Purpose |
|---|---|
| `pixi run setup` | Install/build the fresh SDK and ROS packages |
| `pixi run build` | Rebuild existing dependencies |
| `python scripts/package_model.py --help` | Extract a portable inference bundle without optimizer state |
| `pixi run setup-models` | Install optional model/training interpreter |
| `ros2 launch robot_bringup system.launch.py` | Compose server, visualization and optional components |
| `ros2 run robot_examples robotctl status` | Inspect canonical state |
| `ros2 run robot_examples robotctl sources` | List producer names and instance IDs |
| `robotctl enable`, then `robotctl select NAME` | Explicitly enable and select (use `ros2 run robot_examples` prefix) |
| `robotctl takeover NAME` | Explicit source replacement |
| `robotctl hold/stop/disable/release` | Explicit native control request |
| `robotctl step/reset/capture` | Simulation and acquisition services |
| `pixi run smoke --help` | List individually runnable manual qualification stages |

Bringup arguments: `config`, `mode` (`realtime`/`stepped`), `foxglove`, `record`, `example`, `model`, `model_config`, `model_url`, `instruction`. Starting a recorder does not start a recording. Interactive keyboard teleop runs in its own terminal.

Motion topics are `/robot/command` and `/robot/heartbeat`; outcomes are `/robot/result`. Inspect generated schemas with `ros2 interface show robot_server/msg/MotionCommand` and `ros2 interface show robot_server/srv/Control`.

Authority identity is source name + process UUID + native session generation. Observation identity is runtime UUID + timeline generation + observation sequence. Commands also carry sequence/ID and validity; timed model targets retain their original prediction basis/time.
