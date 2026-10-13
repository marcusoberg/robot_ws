# SO101 deployment resources

`scene.xml` supplies the three-camera scene selected by
`config/so101_rgb3.yaml`. `scene_rgb_depth.xml` supplies the front RGB and
depth configuration selected by `config/so101_rgb_depth.yaml`. Both scenes
include `so101_new_calib.xml` and share the meshes under `assets/`.
`so101_new_calib.urdf` supplies native kinematics and ROS visualization.

The position servos use kp=160 and kv=2. Torque bounds remain ±3.35 Nm,
and joint/controller limits are retained. This tuning came from the historical
simulation qualification; it does not establish physical motor equivalence.
Mesh geometry, camera transforms, lighting and shadow settings retain that
deployment's optical configuration. The new ROS wiring requires its own manual
qualification, described in the workspace developer guide.

The camera calibration labels now describe robot and camera configuration.
Only the labels changed; recorded intrinsics, extrinsics and pixels retain the
original optical settings. Historical recordings retain their original labels.

| Historical label | Current label | Capture plane |
| --- | --- | --- |
| `act-front-v1` | `so101-front-rgb3-v1` | `front/rgb` |
| `act-top-v1` | `so101-top-rgb3-v1` | `top/rgb` |
| `act-wrist-v1` | `so101-wrist-rgb3-v1` | `wrist/rgb` |
| `smolvla-front-v1` | `so101-front-rgb-depth-v1` | `front/rgb`, `front/depth` |

Exports select images using capture plane names. They preserve calibration
labels in export provenance, so historical exports remain usable without
rewriting their metadata. These resources do not select a model or start
recording. Launch instructions belong to the workspace tutorials.
