# Robot Recorder

This ROS package subscribes to Robot Server observations, atomic captures and events. It owns recording sessions and episode labels; Robot Server publishes captures independently of recording.

Launch it with `ros2 launch robot_recorder recording.launch.py`. Recording is inactive until requested. `ros2 run robot_recorder recorderctl --help` lists the commands for starting/stopping recordings, starting/finishing episodes, checking status and awaiting capture acknowledgement. The shared bringup launch can start it together with the robot.

The services `/recording/control` and `/recording/ack` return promptly. Callers may poll or use `recorderctl --wait`; recorder subscription callbacks never wait for disk acknowledgements. A duplicate immutable capture reuses its write ticket. A repeated finalization request reuses its event/operation ticket.

For deterministic scripted episodes, start with an optional explicit `first_capture` identity, and finish with the exact `final_capture` identity. The first identity must not have been written already. Finishing stays pending until that final capture and the finalization event have both been written. Later captures receive no finishing-episode label. Timeline/runtime changes abort the episode, and missing declared final captures cannot produce a successful finish.

Canonical captures and episode events are written to MCAP alongside standard ROS projections. Queue overflow, capture gaps and writer errors mark the recording incomplete. The manifest is finalized on stop; incomplete recordings are excluded from baseline export.
