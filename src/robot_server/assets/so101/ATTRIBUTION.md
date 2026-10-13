# SO101 asset attribution

These robot meshes, calibrated MJCF/URDF and task scenes were copied from
[SO101 SimStudio](https://github.com/rocPAI-Forge/so101-simstudio), revision
`182a038d1d3cdac6729b63b767f438d59ae89ee8`. The upstream project is
Copyright 2026 Alex He and uses Apache-2.0; its [LICENSE](LICENSE) and
[NOTICE](NOTICE) are retained. [PROVENANCE.md](PROVENANCE.md) records local
deployment changes.

The robot design and mesh lineage is
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), under
Apache-2.0. Joint naming and calibrated motor conventions follow
[Hugging Face LeRobot](https://github.com/huggingface/lerobot). The MJCF header
retains the original Onshape CAD export identifier and onshape-to-robot credit.
The gripper collision hulls preserve the upstream CoACD decomposition.

The active scene resources are `scene.xml` and `scene_rgb_depth.xml`. They
share `so101_new_calib.xml`, `so101_new_calib.urdf` and `assets/` meshes.
The ROS resource URLs identify `robot_server`; simulation gains use the
qualified deployment values described in [DEPLOYMENT.md](DEPLOYMENT.md).

The original upstream attribution document is retained as
[UPSTREAM_ATTRIBUTION.md](UPSTREAM_ATTRIBUTION.md) for source history. Its
repository paths describe the upstream tree rather than this package.
