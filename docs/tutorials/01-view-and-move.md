# View and teleoperate PAROL6

With the serial server Ready, start these tasks in separate terminals. Reuse the bridge if it is already running.

```sh
pixi run foxglove
pixi run teleop
```

Teleop starts the robot model and transform publishers. If you already have `pixi run viz` open for the PAROL6, use `pixi run teleop visualize:=false`.

Connect Foxglove to `ws://127.0.0.1:8766` and import the [PAROL6 3D layout](../../src/robot_server/foxglove/parol6_path.json). The model on `/robot/model` follows measured joint positions. In your own 3D panel, set the display frame to `base_link` and enable `/robot/model` and `/teleop/target`.

Run `pixi run engage` in another terminal. Once the leader aligns, move it to see the commanded end-effector pose on `/teleop/target`. Use `pixi run disengage` to pause.
