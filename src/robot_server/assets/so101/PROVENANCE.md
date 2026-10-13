# Pinned SO101 assets

Copied from https://github.com/rocPAI-Forge/so101-simstudio at
`182a038d1d3cdac6729b63b767f438d59ae89ee8`, directory `SO101`.
Robot Studio SO-ARM100/SO101 lineage; see ATTRIBUTION.md, LICENSE, NOTICE and
THIRD_PARTY_NOTICES.md. Original files retain upstream attribution.

Local adaptations:
- Deployment scene.xml sits beside the included robot XML, making mesh paths resolve
  identically in MuJoCo 3.6 and later versions.
- URDF meshes use this ROS package's resource URLs.
- A fixed world-to-base URDF joint matches the MJCF base placement (0.2, -0.12, 0), yaw +pi/2.
- A fixed `gripperframe` URDF link mirrors the MJCF site and is the deployment TCP.
  The original `gripper_frame_link` is retained; its orientation differs by 90 degrees
  and must not be used interchangeably with the MJCF site for Cartesian commands.
- Scene block starts at the upstream fixed-demo pose (0.27, 0.2, 0.0125), yaw -8 degrees.
- Policy cameras preserve the upstream shadow rendering (`render_shadows: true`).
  Disabling shadows made the pinned policy fail the nominal task in a controlled
  comparison; camera appearance is part of this checkpoint's deployment contract.
- Physics uses the scene XML's 1 ms timestep. Upstream's Python evaluator overrides it
  to 1/360 s. This deployment keeps the runtime's integer-nanosecond stepping contract.
- XML formatting changed when adding the deployment transforms; meshes and camera
  extrinsics are unchanged.

Checkpoint: https://huggingface.co/alexhegit/so101-simstudio-lab01-pnp-act-state6
revision `a47afe4dad358031794505799307d4f9756f316c` (Apache-2.0).
Weights are fetched separately and excluded from package installation/version control.
The original model card reports 29/50 successful episodes for its own evaluation;
that result is not a measured success rate for this runtime.
