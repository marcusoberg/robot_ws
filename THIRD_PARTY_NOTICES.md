# Third-party assets and dependencies

The workspace includes robot description and simulation assets with their existing
attribution. This file does not change their licenses or grant a new blanket
license to the workspace.

- **SO101 assets:** copied from SO101 SimStudio at revision
  `182a038d1d3cdac6729b63b767f438d59ae89ee8`. The original Apache-2.0 license,
  notices, asset lineage and local modifications are documented under
  [src/robot_server/assets/so101](src/robot_server/assets/so101/PROVENANCE.md).
- **PAROL6 meshes:** the local source description package identified Source
  Robotics PAROL6 and declared BSD licensing. No separate license text was
  present in that package. The provenance, limitations and file hashes are
  retained in the [mesh README](src/robot_server/assets/parol6/meshes/README.md).
- **ACT checkpoint:** downloaded separately from
  [alexhegit/so101-simstudio-lab01-pnp-act-state6](https://huggingface.co/alexhegit/so101-simstudio-lab01-pnp-act-state6).
  Its upstream card identifies Apache-2.0. The fetch command retains that card;
  no weights or model processor state are distributed in this Git repository.
- **Build dependencies:** Pixi, ROS/RoboStack, MuJoCo, Pinocchio, Coal, Ruckig,
  Boost.SML, nanobind, Torch and LeRobot retain their respective upstream licenses.
  Dependency versions are recorded in the environment locks, native requirements
  and CMake files. Downloaded dependency sources and binaries are not committed.
