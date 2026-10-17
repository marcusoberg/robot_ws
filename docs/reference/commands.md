# Commands and interfaces

Activate from `ws`. `pixi run doctor` checks installation without starting a robot.

| Command | Purpose |
|---|---|
| `pixi run setup` | Install/build the fresh SDK and ROS packages |
| `pixi run build` | Rebuild existing dependencies |
| `pixi run setup-models` | Install optional model/training interpreter |
| `ros2 launch robot_bringup system.launch.py` | Compose server, visualization and optional components |

Bringup arguments: `config`, `mode` (`realtime`/`stepped`), `foxglove`, `model`, `model_config`, `model_url`, `instruction`. Interactive keyboard teleop runs in its own terminal.

Motion topics are `/robot/command` and `/robot/heartbeat`; outcomes are `/robot/result`. Inspect generated schemas with `ros2 interface show robot_server/msg/MotionCommand` and `ros2 interface show robot_server/srv/Control`.

Authority identity is source name + process UUID + native session generation. Observation identity is runtime UUID + timeline generation + observation sequence. Commands also carry sequence/ID and validity; timed model targets retain their original prediction basis/time.

SO101 teleop: tap Space in its launch terminal to activate/pause.
Headless: `ros2 service call /teleop/engage std_srvs/srv/SetBool '{data: true}'`.
Model Client: `ros2 service call /model/engage std_srvs/srv/SetBool '{data: true}'`.
Use `false` to pause. These services submit requests; logs and `/robot/result`
report completion. State is available through `ros2 topic echo /robot/state`.
