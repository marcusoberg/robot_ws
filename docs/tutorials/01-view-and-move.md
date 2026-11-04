# View and teleoperate PAROL6

In a workspace Pixi shell, start the SO101 collection launch:

```sh
pixi run teleop-parol6 port:=/dev/cu.YOUR_LEADER_PORT
```

Connect Foxglove to `ws://127.0.0.1:8766`, use fixed frame `world`, and display
`/robot/cameras/front/rgb`, `/joint_states`, `/robot/reference` and `/teleop/target`.
Run `pixi run engage` in another terminal. Wait for startup alignment; then move the leader.
Run `pixi run disengage` to pause and hold position. Ctrl+C shuts down the launch.

If the server and bridge are already running, add
`start_server:=false foxglove:=false`. The server must use PAROL6, MuJoCo and
`mode:=stepped`. The teleop node requests explicit simulation steps and captures.

See [teleop controls](../../src/robot_teleop/README.md) for recording and headless operation.
