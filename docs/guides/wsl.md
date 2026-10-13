# Windows with WSL2

Use **WSL2 with Ubuntu 24.04 on an Intel/AMD x86_64 PC**. All workspace commands,
ROS nodes, C++ compilation and Python bindings run inside Ubuntu. Foxglove can run
as a Windows application and connect to the bridge in WSL.

WSL2 uses this project's existing `linux-64` Pixi lock and Linux Python wheels.
The compiler and GNU Make come from Pixi; Visual Studio and a Windows Python
installation are not needed for this build. Native Windows and WSL on Windows ARM64 are not covered
by the current locks. There is no end-to-end WSL qualification yet. The repository's
Linux CI checks compilation/imports, not WSLg rendering or robot task success.

## Prepare Windows and Ubuntu

In **Windows PowerShell as Administrator**, install WSL if needed:

```powershell
wsl --install -d Ubuntu-24.04
```

Follow the prompts, restart Windows if requested, and create your Ubuntu user.
For an existing installation, update WSL and confirm the distribution uses version 2:

```powershell
wsl --update
wsl --list --verbose
```

If that list shows version 1 for Ubuntu-24.04, convert that distribution with
`wsl --set-version Ubuntu-24.04 2`. See Microsoft's [WSL installation guide](https://learn.microsoft.com/en-us/windows/wsl/install).

In the **Ubuntu terminal**:

```sh
sudo apt update
sudo apt install -y git curl ca-certificates libgl1 libegl1 libopengl0 mesa-utils
mkdir -p ~/PROJECTS
cd ~/PROJECTS
git clone https://github.com/marcusoberg/robot_ws.git
cd robot_ws
./scripts/setup.sh --local-pixi --jobs 2
.tools/pixi shell
pixi run doctor
```

Expect `"errors": []`. Setup compiles an installed C++ consumer and imports the
native Python extension without creating a robot. The `--jobs` setting bounds
compiler parallelism; ROS packages build one at a time. If compilation is killed
for lack of memory, retry with `pixi run build --jobs 1` after setup dependencies
have installed, or use `--jobs 1` on the initial setup command.

Keep the clone inside the **Linux filesystem**, such as `~/PROJECTS/robot_ws`.
Microsoft recommends this for [Linux filesystem performance](https://learn.microsoft.com/en-us/windows/wsl/filesystems).
Build artifacts also contain machine-specific paths and native binaries, so use a
fresh clone instead of sharing `.pixi`, `.workspace`, `build` or `install` with
Windows or another operating system. The repository enforces LF line endings,
including extensionless ROS executable scripts.

Do not activate a system Conda environment or source `/opt/ros/.../setup.bash` over
Pixi. RoboStack supplies the ROS installation for this workspace.

## Camera rendering through WSLg

Use an updated WSL2 installation with WSLg and your GPU vendor's current Windows
driver. WSLg supplies the Linux display used by MuJoCo's GLFW renderer. Microsoft's
[GUI application guide](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
describes the supported Windows versions and drivers.

Inside Ubuntu, a useful graphics check is:

```sh
glxinfo -B
```

This should report a renderer; it is a graphics diagnostic, not a robot test.
In the activated **Robot Server terminal**, choose GLFW explicitly before launching:

```sh
export MUJOCO_GL=glfw
ros2 launch robot_bringup so101_act.launch.py
```

Do not select EGL solely because inference uses CUDA. Graphics rendering and Torch
compute are separate, and a working CUDA device does not establish EGL support.
If WSLg is unavailable, MuJoCo also supports software rendering through OSMesa:
install `libosmesa6` in Ubuntu and set `MUJOCO_GL=osmesa` before starting the server.
Software rendering can be too slow for the intended camera/inference rate. EGL is
another option only when that machine has a working EGL implementation. These
choices follow [MuJoCo's renderer selection](https://github.com/google-deepmind/mujoco/blob/3.10.0/python/mujoco/rendering/classic/gl_context.py).

In Windows Foxglove, connect to **`ws://127.0.0.1:8766`** and import the SO101 layout
as described in the main README. Windows can normally access WSL services through
[localhost forwarding](https://learn.microsoft.com/en-us/windows/wsl/networking).
Keep all ROS nodes in the same Ubuntu distribution; Foxglove uses WebSockets, so
Windows does not need ROS discovery or a separate ROS installation.

If Foxglove cannot connect, first confirm the bridge is listening inside Ubuntu:
`ss -ltn | rg 8766` (use `grep` if `rg` is unavailable). Check WSL's networking and
localhost-forwarding configuration. There is no need to expose the model or bridge
on a public network.

## ACT inference

Follow the [main README](../../README.md#2-install-model-dependencies-and-fetch-act)
for model setup, fetching and the five-terminal sequence. Every terminal except
Windows Foxglove should be an Ubuntu terminal with this clone's Pixi environment.

Use **`--device cuda`** when your NVIDIA GPU is available to WSL, or **`--device cpu`**
otherwise. MPS is specific to macOS. With NVIDIA, install/update the **Windows**
driver; do not install a Linux NVIDIA display driver inside WSL. See NVIDIA's
[CUDA on WSL guide](https://docs.nvidia.com/cuda/wsl-user-guide/index.html).
The locked Torch packages supply their userspace CUDA dependencies.

After `pixi run setup-models`, check Torch in its own environment:

```sh
environments/models/.venv/bin/python -c \
  'import torch; print(torch.__version__); print("CUDA available:", torch.cuda.is_available())'
```

Then the Model Server command in **terminal B** is:

```sh
pixi run model-server --model act \
  --checkpoint models/act-base/model \
  --features src/model_client/config/act.features.json \
  --revision a47afe4dad358031794505799307d4f9756f316c \
  --device cuda --offline
```

Replace `cuda` with `cpu` when necessary. Start the result monitor before issuing
explicit enable/select commands in another terminal, as in the main walkthrough.
CPU inference and software rendering may lead to late predictions; inspect actual
native rejection reasons instead of increasing motion limits.

USB/serial hardware access is separate from this simulation setup and needs WSL
USB forwarding. Physical hardware and physical-camera operation remain unqualified.
