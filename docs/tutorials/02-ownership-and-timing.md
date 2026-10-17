# Control ownership and timing

`select` identifies a producer, not a robot. Producer activation now performs
enable/acquire inside Robot Server, through the SDK. Teleop Space and the
`/model/engage` service request activation directly. Startup never activates motion.

Only one producer can own a native session. Pause the current producer before
activating another. A reset, producer disconnect or expired session cannot restart
motion without another explicit activation. Observe `/robot/result` for correlated
completed, rejected or canceled operations; queued is only a receipt.

For manual low-level diagnostics inspect `ros2 interface show robot_server/srv/Control`.
Actions ACTIVATE (11) and DEACTIVATE (12) require a live producer's name and process
UUID from `/robot/state`. Every request needs a new UUID `request_key`. The server
returns a request ID; final results use that ID on `/robot/result`.

`/robot/step`, `/robot/reset` and `/robot/capture_once` remain direct simulation
services. Inspect their schemas with `ros2 interface show robot_server/srv/Step`
(and `Reset` or `CaptureOnce`). In stepped mode requests execute during explicit
steps. The SO101 node supplies these steps; no CLI silently advances simulation.
