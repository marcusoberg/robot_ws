# PAROL6 path source

`robot_path` generates a Cartesian route, solves Pinocchio IK, and publishes joint
angle goals through the robot server's normal `JointGoal` interface. It contains
no serial connection, homing implementation, simulator execution or motor control.

The robot server selects the simulation or serial backend, validates commands,
owns source sessions, and holds/disables execution when a source fails or its
commands expire. Homing and E-stop monitoring belong to that server/backend.

## Run

From the workspace root, in separate terminals:

```sh
pixi run path
pixi run foxglove
```

The first task launches a robot-only PAROL6 simulation and starts tracing after
planning completes. The log reports the planned execution duration.
`pixi run disengage` requests a pause. After stopping, restart the
source to plan from its new start pose. The bridge remains a separate task.

The scene contains the robot, floor, lighting and a viewer viewpoint. It contains
no cube, container or pick-and-place task. No image rendering worker or camera
publishers are created. Foxglove displays the robot through `/joint_states`, TF
and `/robot/urdf`; `/path/markers` displays the planned shapes and transfers.
The viewer camera defined in the scene is a viewpoint, not a robot sensor.

Import `src/robot_server/foxglove/parol6_path.json` into Foxglove for a 3D panel
with the robot mesh markers (`/robot/model`) and route (`/path/markers`) enabled.
In an existing 3D panel, enable `/robot/model` under Topics. The visualization
launch publishes each visual mesh in its own TF frame, so it follows joint motion.
`/robot/urdf` remains available for an optional URDF layer. The
separate `foxglove` task permits the model's uppercase `.STL` mesh filenames.
Restart the bridge after updating to this task.

The stock initial homing ceilings for J1–J6 are approximately
18.02, 7.17, 7.93, 78.05, 71.02 and 53.72 degrees/s. They come from firmware
revision `1e7fa639`, `main.cpp`, converted using the stock gearing.

## Geometry

All coordinates are metres in the URDF `base_link` frame:

| Setting | Default | Meaning |
| --- | --- | --- |
| `width` | 0.18 | Line length, triangle side and circle diameter |
| `height` | 0.16 | Shape centre above the base plane |
| `distance` | 0.30 | Panel offset from the base axis |
| `speed` | 1.0 | Fraction of stock initial homing joint rates; range (0, 1] |
| `tip` | `tool_tcp` | URDF point whose position traces the route |

The line is on the left (`y=+distance`), the triangle in front (`x=distance`),
and the circle on the right (`y=-distance`). The two side planes are parallel.
Transfers follow arcs around the base. The bottom of the line/circle is 70 mm
above the base; their top is 250 mm. The triangle reaches about 264 mm. These
extents exclude the initial transfer from the robot's starting pose.

IK constrains position and leaves tool orientation free. This demo traces in
space; it does not enforce a pen orientation against a surface. Every knot and
joint-interpolated midpoint is checked against the requested Cartesian route.
The planner times each segment against the six initial homing rates and a
0.30 rad/s² acceleration limit. These are joint speed ceilings, not a constant
Cartesian speed: simultaneous tracing differs from the staged homing sequence.
`speed:=0.5` doubles the duration and reduces acceleration by four. The server
independently owns execution deadlines and backend motion limits. IK checks do
not constitute physical collision clearance.

```sh
pixi run path width:=0.18 height:=0.16 distance:=0.30
pixi run path speed:=0.5
pixi run path gripper:=msg_ai_100
pixi run path-preview
```

`path-preview` performs offline IK only. Launch arguments configure the ROS node;
its equivalent standalone inspection flags are in `python -m robot_path.preview --help`.

## Code map

- `settings.py`: named dimensions, rates and tolerances, with SI units.
- `geometry.py`: line, circle and orbital-transfer geometry.
- `planner.py`: IK checking and joint trajectory time parameterisation; uses `robot_kinematics` directly, with no teleop dependency.
- `markers.py`: visualization messages only.
- `node.py`: ROS source ownership, engage-topic requests and joint-goal publishing.
- `preview.py`: offline planning inspection.
- `joint_move.py`, `move_joint.py`: timed joint moves and their ROS command source.
- `launch/parol6_path.launch.py`: robot-only simulator plus source; no bridge.

`node.py` is the actual executable installed as `path_node.py`. There is no thin
extensionless wrapper in a `scripts` folder. Exceptions propagate to the outer
node lifecycle, which requests server pause before shutting down. The server's
own source lease also expires if the producer disappears or cannot send a stop.

## Joint poses

With an already homed server in `Ready`, return to the post-homing pose:

```sh
pixi run standby
```

This moves to the firmware's standby counters, approximately
`[90, -90, 180, 0, 0, 180]` degrees, in four seconds. It is a joint move and does
not run the sensor-seeking homing sequence. `pixi run home` remains the homing command.

For `MoveJoint(90.0,-144.683,108.171,5.0,0.0,90.0,t=4)`:

```sh
pixi run pose
```

Both tasks start the move immediately, preserve the gripper position, and exit
after the server confirms the endpoint and holds in `Ready`. If a path or teleop
source owns the server, pause it first with `pixi run pause`.

The commands use a smooth quintic joint trajectory with zero velocity and
acceleration at both endpoints, following Commander's timed `MoveJoint` mode.
They publish at the native execution cadence and retain the requested duration.
Position and velocity feed-forward are sent together, as in Commander's joint
trajectory mode. After installing this feature, fully restart the robot server
to load the updated command message and native runtime.
For another duration or target, use `--time` or the underlying command:

```sh
pixi run standby --time 4
pixi run -e ros python -m robot_path.move_joint --degrees 90 -144.683 108.171 5 0 90 --time 4
```

## Serial robot

The path source is identical for simulation and serial. Configure and start the
robot server separately, with the correct device and an unloaded, isolated tool:

```sh
pixi run server-parol6-serial serial_port:=/dev/cu.usbmodemYOUR_DEVICE tool_isolated:=true
```

This opens only the selected port and sends disabled polling. OUT1/OUT2 are LOW
and gripper control fields zero; this backend currently controls only the six
arm joints. After observing a physical E-stop press/release in the server's
telemetry, run `pixi run home`. It waits until homing completes and the enabled arm is
holding in `Ready`, then:

```sh
pixi run path-source
pixi run engage
```

The path package never homes or enables a transport directly. The server refuses
activation before homing on this connection. Native polling remains active during
homing, tracking and idle states. Normal pause holds with command 255 and zero
velocity; homing cancellation, disable, E-stop and faults send command 102 and
invalidate homing. After disable/fault, `pixi run reset` opens a new disabled
connection before another homing. See `src/robot_server/SERIAL.md` for protocol
semantics and the scope of verification.

`/robot/serial_telemetry` exposes the returned packet fields, including E-stop,
homing flags, counters and raw payload. Feedback is labelled as a step-count
estimate in `/robot/ownership`; it is not encoder measurement. The firmware has
no per-target sequence acknowledgement, so physical `delivered_sequence` remains
zero. The passive gripper state is unmeasured. There are no serial cameras.

Stock firmware command definitions and conversions come from
[main firmware revision 1e7fa639](https://github.com/Source-Robotics/PAROL6-Desktop-robot-arm/tree/1e7fa6391440660a5c4ce2166e8a33e7e3d334cf/PAROL6%20control%20board%20main%20software/src).
The USB handshake cannot identify the installed firmware version. Stock E-stop
is host-mediated: it depends on a functioning host and USB link. Native deadlines
cannot guarantee delivery of a disable after USB loss, host/process death or OS
suspension. Keep direct power isolation accessible during physical validation.
Physical operation of this server integration has not been validated on an arm.
The `path-source` task uses the path's configured timing. The serial backend sends
the planned joint-position targets directly; joint-position bounds and the
tracking-error check still apply.
