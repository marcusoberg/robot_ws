# Teleoperation

From the workspace Pixi shell:

```sh
ros2 launch robot_teleop so101_parol6.launch.py port:=/dev/cu.YOUR_LEADER_PORT
```

This starts the stepped PAROL6 simulation, Foxglove bridge and SO101 leader node.
To control the SO101 simulation instead, stop the existing server and teleop first:

```sh
ros2 launch robot_teleop so101_so101.launch.py port:=/dev/cu.YOUR_LEADER_PORT
```

This uses joint-to-joint targets and a continuous gripper, with the SO101 scene's
three RGB cameras. Its recordings contain six joint positions/actions in radians;
PAROL6 recordings retain six arm joints plus a binary gripper action.
Both launches use the upstream leader model in `assets/so101_leader`, rooted at
`base_link`. PAROL6 maps the actual handle frame with no added tool rotation;
SO101's scene mount is applied only to the target preview. Mapping is configured
in the corresponding file under `config/`.

For an existing server use `start_server:=false foxglove:=false`.
Tap Space in the teleop terminal to activate or pause. Startup is paused.
Activation directly asks Robot Server to enable and acquire this producer's native
session; another producer is never taken over. Wait for `Active` before moving.
Pause cancels demands, releases authority and holds the arm; it does not remove
motor holding force. Reset, leader loss and authority loss require another tap.

`keyboard:=off` disables terminal input. The same operation is exposed as:

```sh
ros2 service call /teleop/engage std_srvs/srv/SetBool '{data: true}'
ros2 service call /teleop/engage std_srvs/srv/SetBool '{data: false}'
```

The service acknowledges the request; `/teleop/status` and the terminal report the
actual result. A paused recording is invalid and must be discarded.

```sh
ros2 service call /teleop/record/start std_srvs/srv/Trigger '{}'
ros2 service call /teleop/record/save std_srvs/srv/Trigger '{}'
ros2 service call /teleop/record/discard std_srvs/srv/Trigger '{}'
```

Save/discard pause control. Wait for the written acknowledgement on `/teleop/status`.
Datasets are written under `training/datasets` by default.

The separate joint keyboard producer remains available as `ros2 run robot_teleop keyboard`:
`d` activates/pauses, number keys choose a joint, `j/k` adjust its target, Space is
its clutch, and `q` exits. It is intended for a realtime server.
