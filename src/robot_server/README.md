# Robot server

This ROS node opens the native robot SDK, publishes robot state and cameras, and
accepts commands from the explicitly selected input source. C++ owns execution,
interpolation, inverse kinematics, authority and backend timing. The Python node
handles ROS messages and visualization outside that execution lane.

The native SDK is installed independently. This package does not start a robot
WebSocket server or an HTTP experiment manager. Recording and episode labels
belong to `robot_recorder`; policies and input devices publish ROS commands.

From the shared workspace environment:

```sh
ros2 launch robot_server server.launch.py mode:=stepped
```

The default configuration uses SO101 and three 640 × 480 RGB cameras. Choose
`config/so101_rgb_depth.yaml` for one 512 × 512 RGB camera with a separate depth
plane, or `config/parol6.yaml` for the PAROL6 simulation. Deployment resources
are installed in this package. The server starts disabled, with no selected
source. Enable explicitly, then select a source. In stepped simulation, queued
control operations apply only when the caller explicitly advances simulation.

`/robot/control` queues lifecycle and source-selection requests. Its response is
an acknowledgement; `/robot/result` reports the actual outcome. A new control,
step, reset or capture request needs a fresh UUID in `request_key`; retrying the
same request UUID does not execute it again. `/robot/step` emits a capture for
every requested action interval and returns the last identity. It does not wait
for recording. Collection programs wait separately for the recorder's write
acknowledgement before advancing again.

Actors publish `/robot/heartbeat` to announce a source name and process UUID.
Only heartbeats from the selected native session renew its authority. Commands
include the server's runtime/timeline/session identity, the observation that
provides their freshness basis, an increasing producer sequence and a command
ID. Stale, unselected or malformed input is rejected. Missing heartbeat revokes
authority; a source never reacquires automatically. A deadman or clutch pause
requests a native controlled brake and retains the selected session while its
heartbeat remains healthy. Fresh valid demand resumes through native admission.

Timed target chunks contain original position samples and timestamps. Model
programs do not fit paths. The native runtime prepares, checks and admits the
future path; `/robot/result` distinguishes preparation from admission. The raw
prediction origin remains unchanged.

`msg/` and `srv/` contain the interface definitions. `Observation.record_json`
preserves the original SDK observation and adds `available_sources`,
`selected_source` and server status. `Capture.packet` is a bounded compressed
MessagePack envelope containing `robot.capture.v2`, the original SDK observation
and named immutable pixel arrays. It contains no episode or recording state.
Standard joint, TF and camera topics are visualization projections of these
records. Foxglove layouts are installed under `foxglove/`: `so101_rgb3.json`,
`so101_rgb_depth.json` and `parol6.json` match the three configurations.

The supplied camera resources are simulation cameras. Physical operation and
physical camera transport remain unqualified.

`robot_server_node` contains the node implementation. `robot_server_client` is a small ROS-only producer helper for source identity, command envelopes and heartbeats. It never imports the native SDK or acquires authority. Model Client, teleop and examples use it to speak the same interface without depending on each other's packages.
