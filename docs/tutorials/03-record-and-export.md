# Record and export

> Recording your own run uses the included code. Commands below that refer to `artifacts/legacy-20261011` are optional historical comparisons and require separately preserved local recordings; those recordings are not included in the repository.

Start `system.launch.py mode:=stepped record:=true`. Enable and step, select your producer and step, using the explicit sequence from tutorial 2.

The short recording helper starts its own episode and explicitly requests ten intervals:

```sh
pixi run smoke record --steps 10 --output runs/my-first-recording
```

It requests capture → waits for its writer acknowledgement → requests the next step. It never enables or selects a source. The Robot Server has no recording dependency. The default episode is unapproved; add `--approved` only after inspecting what you recorded. A short capture smoke does not establish task success.

Inspect independent lifecycle operations:

```sh
ros2 run robot_recorder recorderctl status
ros2 topic echo /recording/status
```

Re-export the preserved recordings to validate the historical 8/2 split:

```sh
pixi run setup-models
pixi run export-dataset --run artifacts/legacy-20261011/robot-experiments/runs/so101-act-final --output runs/act-reexport --config src/model_client/config/act.yaml
pixi run export-dataset --run artifacts/legacy-20261011/robot-experiments/runs/so101-smolvla-final --output runs/smolvla-reexport --config src/model_client/config/smolvla.yaml
```

Inspect alignment without LeRobot or exporting pixels:

```sh
recording-export --run artifacts/legacy-20261011/robot-experiments/runs/so101-act-final --config src/model_client/config/act.yaml --alignment-only
```

Expect eight training episodes and two validation episodes. Export uses measured state at t and the admitted reference at t+dt. Irregular live resampling is not implemented.
