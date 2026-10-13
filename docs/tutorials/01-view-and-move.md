# View and move a simulated robot

Use separate terminals, each activated with `pixi shell` from `ws`.

1. Launch the robot:

```sh
ros2 launch robot_bringup so101_act.launch.py
```

Open Foxglove and connect to `ws://127.0.0.1:8766`. Inspect `/joint_states`, `/tf`, and `/robot/cameras/*`. The state begins `DISABLED`, without a selected source.

2. In terminal two, start a readable producer:

```sh
ros2 run robot_examples send_joint --joint 0 --offset 0.03 --finite
```

It advertises itself as `example`. It sends no motion until selected.

3. In terminal three, start the result monitor first and leave it running:

```sh
ros2 topic echo /robot/result
```

In terminal four, inspect the producer and explicitly enable/select it:

```sh
pixi run robotctl sources --timeout 30
pixi run robotctl enable --timeout 30
pixi run robotctl select example --timeout 30
```

The producer sends one finite joint trajectory and keeps its heartbeat active. C++ generates and executes the motion. Look for accepted then completed results and a reference/measurement change in `/robot/state`. A result saying queued is only receipt, not execution.

4. Finish explicitly:

```sh
ros2 run robot_examples robotctl hold
ros2 run robot_examples robotctl stop
ros2 run robot_examples robotctl disable
```

Use Ctrl-C to stop producer and launch terminals. Launching or stopping a producer does not enable the robot.

For PAROL6 simulation use `ros2 launch robot_bringup parol6.launch.py`. This walkthrough does not qualify physical hardware.

You can use `pixi run smoke view --output runs/my-view` and `pixi run smoke move --output runs/my-move` for the same launch/finite publisher, with a process-exit report after Ctrl-C. Enable and selection remain separate commands. Save `/robot/result` with `ros2 topic echo /robot/result > runs/my-move/results.yaml`, and record the state/visual observations you checked. Process exit alone is not a pass.
