# Fresh workspace implementation

> Historical migration snapshot from 2026-10-12. References to uninitialized workspace Git, local archives, build reports and operational status describe that migration, not the current repository distribution. Those local artifacts are excluded from Git; see the main README for current setup.

The active SDK is curated from the original C++ execution dependency closure. Native dependencies retain Pinocchio 3.8.0, Coal 3.0.2, Ruckig 0.15.3, SML 1.2.0, MuJoCo 3.10.0 and Nanobind 3.1.0. `scripts/setup.py` builds it into `install/runtime`, compiles an installed C++ consumer without running it, installs the Python facade, and builds only the six explicit ROS package paths.

The previous root generated outputs were archived. Four retired folders were relocated intact, retaining their Git metadata. No workspace repository/submodule was initialized. Artifacts were copied independently using APFS copy-on-write clones, verified by SHA-256 before relocation, and recorded in `artifacts/legacy-20261011/inventory.json`. These are separate files, not hard links. Six broken original SmolVLA base-checkpoint links are explicitly missing; complete trained checkpoints and training states are retained.

`build/qualification/doctor.json` records static imports/resource provenance. `build/qualification/integrity.json` compares the complete original runtime tree with its pre-migration baseline. A lint tool accidentally wrote six new cache entries in the original working directory; exactly those added entries were removed. All source, Git and pre-existing generated/environment files match the baseline. Two Finder `.DS_Store` metadata files differ and are reported separately; no byte-identical claim is made for those metadata files.

Compilation and static loading do not qualify robot operation or model task performance. The operational stages, native fitting comparison, recording/export and training/resume remain user-run. See `manual-qualification.md` and the tutorials. The optional model environment has a lock file; its heavy dependencies are installed only when the user runs `pixi run setup-models`.

On macOS, setup refreshes generated extensions before applying install RPATH changes so repeated installation is idempotent. The final build has no installation errors. Code-signing warnings from RoboStack remain informational; generated type-support imports pass. Operational ROS communication remains a manual stage. Linux serial/backend builds, physical hardware/camera transport, live irregular resampling and MPS training are unqualified.

The final static result is `build/qualification/implementation.json`; detailed imports and build output are in that same directory. No operational result has been recorded under `runs`.
