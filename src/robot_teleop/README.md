# SO101 leader teleoperation

Run `pixi run teleop` with a Ready PAROL6 server. From another terminal, `pixi run engage` starts motion and `pixi run disengage` requests a hold. Run one producer at a time.

Teleop launches the selected follower's visual model and TF. If `pixi run viz` is already running for that follower, use `pixi run teleop visualize:=false` to reuse it. Start the Foxglove bridge once with `pixi run foxglove`. Connect to `ws://127.0.0.1:8766` and open the [PAROL6 3D layout](../robot_server/foxglove/parol6_path.json). In a custom layout, select `base_link` as the display frame and enable `/robot/model` and `/teleop/target`.

The model follows the server's measured `/joint_states`. After startup alignment, teleop publishes the exact commanded end-effector pose on `/teleop/target` as a `geometry_msgs/msg/PoseStamped`, with the same frame and sample timestamp as the control message. Disengaging stops goal updates.

On first engagement in a server episode, teleop requests `activate_teleop`. The server executes the configured four-second joint move via the PAROL6 API. Once the API confirms completion, teleop aligns the current SO101 end-effector to the measured follower TCP in server status. Engaging again anchors at the current poses, so moving the leader while paused introduces no target jump.

At each fresh leader sample (nominally 20 Hz), `robot_teleop/leader_pose.py` decodes the calibrated encoders and applies leader FK and the relative frame mapping. The node publishes the resulting Cartesian pose on `/robot/pose_goal`. A stale leader or missing server status requests a pause. The serial server owns follower IK and streams joint references through the API.

Leader port, calibration, frame mapping and startup selection are in `robot_teleop/teleop.toml`. Robot TCP frames, per-gripper tracking weights and the start pose are in `external/robot_core/robot.toml`.
