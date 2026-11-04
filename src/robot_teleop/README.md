# SO101 leader teleoperation

`pixi run teleop` starts the configured SO101-to-PAROL6 source. In another terminal, use `pixi run engage` to activate it and `pixi run disengage` to pause. These publish a Bool on `/teleop/engage`. Recording is deferred on the Python API branch. There is no terminal keyboard handler or direct-joint teleop mode.

The PAROL6 configuration preserves the four-second startup move to `(90, -144.683, 108.171, 5, 0, 90)` degrees. Once feedback confirms the endpoint, the current leader TCP becomes the reference frame. Later engagements in the same episode keep that alignment. The SO101 follower configuration starts tracking directly, without a startup move.

## Code map

- `scripts/so101.py`: the complete ROS node and executable. It owns status/session callbacks, startup coordination, the IK worker, target publication, and shutdown.
- `robot_teleop/leader.py`: calibrated serial acquisition from the SO101 leader.
- `robot_teleop/fk_ik_map.py`: `FKIKMapping`, encoder conversion and application-specific leader/follower frame and gripper mapping.
- `../../external/robot_kinematics`: reusable ROS-free Pinocchio FK/IK and joint smoothing. Both teleop and `robot_path` import this library directly.
- `config/*_ik.toml`: robot frames, joint ordering, startup pose, observation contract, mapping and numerical IK parameters.
- `launch/so101_so101.launch.py`: starts `so101.py` directly. `ik_config` defaults to the SO101 config; the PAROL6 Pixi tasks supply their own config.

The leader is sampled at 20 Hz. Completed IK targets are delivered asynchronously; a 100 Hz ROS timer collects results and updates motion. The API receives live IK targets through `servo_j` and owns its smoothing/dispatch clock. Timed startup uses `move_j`. The simulation node is currently an inactive placeholder and publishes no status.

Missing/stale feedback pauses the source. Configuration and dependency failures propagate directly. There are no import fallbacks, automatic solver selection or alternate joint mappings. The API owns controller limits and E-stop handling; the ROS server retains session/deadline checks.

The PAROL6 configs explicitly use `observation.cameras = []` for the robot-only server. SO101 uses the three-camera observation contract. Set this contract to match the server; the source does not guess a replacement contract from a mismatch.
