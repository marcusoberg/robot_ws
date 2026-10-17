# SO101 leader model

Unmodified URDF and referenced meshes from TheRobotStudio/SO-ARM100,
revision `a758567c3978dfeefe282ede0500085a48fe8f78`, `Simulation/SO101/so101_leader_new_calib.urdf`.
https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101

The root is `base_link`; there is no scene/world mounting transform.
`handle_frame_link` is the physical handle reference. `gripper_frame_link`
is the upstream virtual follower tool reference, useful for matching SO101 FK.
The five arm joints use the midpoint calibration convention. The trigger's
URDF range is provisional: teleop uses LeRobot's calibrated 0–100 reading,
not the provisional trigger angle. See LICENSE for the upstream terms.
