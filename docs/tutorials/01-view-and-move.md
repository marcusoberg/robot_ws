# View and teleoperate PAROL6

In a workspace Pixi shell, start the SO101 collection launch:

```sh
ros2 launch robot_teleop so101_parol6.launch.py port:=/dev/cu.YOUR_LEADER_PORT
```

Connect Foxglove to `ws://127.0.0.1:8766`, use fixed frame `world`, and display
`/robot/cameras/front/rgb`, `/joint_states`, `/robot/reference` and `/teleop/target`.
Tap Space in the launch terminal. Wait for `Active`; then move the leader.
Tap Space again to pause and hold position. Ctrl+C shuts down the launch.

If the server and bridge are already running, add
`start_server:=false foxglove:=false`. The server must use PAROL6, MuJoCo and
`mode:=stepped`. The teleop node requests explicit simulation steps and captures.

See [teleop controls](../../src/robot_teleop/README.md) for recording and headless operation.
