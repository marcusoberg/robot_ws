# Set up the workspace

The [main README](../../README.md#1-clone-and-build) is the complete setup and
public ACT walkthrough. Clone the repository, then run from its root:

```sh
./scripts/setup.sh
# Alternatively, install Pixi in this workspace:
# ./scripts/setup.sh --local-pixi
pixi shell
pixi run doctor
```

For a local Pixi installation enter with `.tools/pixi shell`. Open a fresh shell
after building so the installed ROS packages are on the package path.

Setup installs pinned SDK dependencies, compiles C++/bindings and builds only the
six maintained ROS packages. Generated files live in `build`, `install`, `log`,
`.workspace` and `.pixi`. It does not initialize Git or run robot/model smoke tests.

Rebuild after editing: `pixi run build`. Optional model dependencies:
`pixi run setup-models`. Fetch the public ACT inference files: `pixi run fetch-act`.
Robot viewing and direct control do not require LeRobot or Torch.

Activate from this clone's root. Do not source an archived ROS overlay or activate
a second Conda environment over Pixi. If a command resolves old code, leave the
shell and enter a fresh workspace Pixi shell.
