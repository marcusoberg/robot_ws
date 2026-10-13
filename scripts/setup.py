"""Build the new SDK and explicitly selected ROS packages from the workspace root."""

from pathlib import Path
import argparse
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGES = (
    "robot_server",
    "robot_examples",
    "robot_teleop",
    "model_client",
    "robot_recorder",
    "robot_bringup",
)


def run(*command, **kw):
    subprocess.run([str(c) for c in command], check=True, **kw)


def positive_jobs(value):
    jobs = int(value)
    if jobs < 1:
        raise argparse.ArgumentTypeError("--jobs must be a positive integer")
    return jobs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument(
        "--jobs",
        type=positive_jobs,
        default=2,
        help="Maximum simultaneous compiler jobs (default: 2)",
    )
    args = parser.parse_args()
    sdk = ROOT / "external/robot_runtime"
    env = {
        **os.environ,
        "ROBOT_RUNTIME_NATIVE_ENV": str(ROOT / ".workspace/native-env"),
        "ROBOT_RUNTIME_BUILD": str(ROOT / "build/runtime"),
        "ROBOT_RUNTIME_INSTALL": str(ROOT / "install/runtime"),
        "ROBOT_PYTHON": sys.executable,
        "CMAKE_BUILD_PARALLEL_LEVEL": str(args.jobs),
        # colcon otherwise gives each Makefile package all detected CPUs.
        "MAKEFLAGS": f"-j{args.jobs}",
    }
    if not args.build_only:
        run(sdk / "scripts/setup_native.sh", env=env)
        for name in ("model_api", "recording_tools"):
            run(
                sys.executable,
                "-m",
                "pip",
                "install",
                "--no-deps",
                "--no-build-isolation",
                "-e",
                ROOT / "python" / name,
            )
    run(
        "cmake",
        "-S",
        sdk,
        "-B",
        ROOT / "build/runtime",
        "-G",
        "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_INSTALL_PREFIX={ROOT / 'install/runtime'}",
        f"-DPython_EXECUTABLE={sys.executable}",
        f"-DPython3_EXECUTABLE={sys.executable}",
        f"-DROBOT_RUNTIME_NATIVE_SDK={ROOT / '.workspace/native-env'}",
        "-DROBOT_RUNTIME_BUILD_PYTHON=ON",
        "-DROBOT_RUNTIME_BUILD_MUJOCO=ON",
        f"-DROBOT_RUNTIME_BUILD_SERIAL={'ON' if sys.platform.startswith('linux') else 'OFF'}",
        f"-Dnanobind_DIR={ROOT / '.workspace/native-env/lib/python3.12/site-packages/nanobind/cmake'}",
        env=env,
    )
    run("cmake", "--build", ROOT / "build/runtime", "--parallel", str(args.jobs), env=env)
    if sys.platform == "darwin":
        # Recopy generated extensions before CMake applies their install RPATH.
        # Otherwise a repeated install patches an already-patched up-to-date file.
        for binary in (ROOT / "install/runtime/python/robot_runtime_native").glob(
            "_native*.so"
        ):
            binary.unlink()
    run("cmake", "--install", ROOT / "build/runtime", env=env)
    # Compile/link the public installed C++ SDK facade; never run the consumer.
    run(
        "cmake",
        "-S",
        ROOT / "scripts/sdk_consumer",
        "-B",
        ROOT / "build/sdk-consumer",
        "-G",
        "Ninja",
        f"-DCMAKE_PREFIX_PATH={ROOT / 'install/runtime'}",
        env=env,
    )
    run("cmake", "--build", ROOT / "build/sdk-consumer", "--parallel", str(args.jobs), env=env)
    # CMake installs the extension; pip installs its direct Python facade.
    run(
        sys.executable,
        "-m",
        "pip",
        "install",
        "--no-deps",
        "--no-build-isolation",
        "-e",
        sdk / "python",
        env=env,
    )
    # Import in a new process so Linux loader/ABI errors fail setup immediately.
    # This registers binding types; it does not construct a Robot or render images.
    binding_env = {**env, "PYTHONPATH": str(ROOT / "install/runtime/python")}
    run(
        sys.executable,
        "-c",
        "from robot_runtime_native import _native; "
        "print('Native bindings:', _native.__file__)",
        env=binding_env,
    )
    if sys.platform == "darwin":
        for package in ("robot_server", "robot_recorder"):
            for binary in (
                ROOT / "install/ros/lib/python3.12/site-packages" / package
            ).glob(f"{package}_s__*.so"):
                binary.unlink()
            binary = (
                ROOT / "install/ros/lib" / f"lib{package}__rosidl_generator_py.dylib"
            )
            if binary.exists():
                binary.unlink()
    # Ignore stale root colcon_defaults.yaml; the complete build paths are explicit.
    colcon_env = {
        **env,
        "COLCON_DEFAULTS_FILE": str(ROOT / "scripts/colcon_defaults.yaml"),
    }
    run(
        "colcon",
        "--log-base",
        ROOT / "log/ros",
        "build",
        "--base-paths",
        *(ROOT / "src" / name for name in ROS_PACKAGES),
        "--build-base",
        ROOT / "build/ros",
        "--install-base",
        ROOT / "install/ros",
        "--merge-install",
        "--executor",
        "sequential",
        "--cmake-args",
        f"-DPython3_EXECUTABLE={sys.executable}",
        f"-DPython_EXECUTABLE={sys.executable}",
        cwd=ROOT,
        env=colcon_env,
    )
    if sys.platform == "darwin":
        prefix = Path(sys.prefix) / "lib"
        for name in ("robot_server", "robot_recorder"):
            for library in (ROOT / "install/ros/lib").glob(f"lib{name}*.dylib"):
                link = prefix / library.name
                if link.is_symlink():
                    link.unlink()
                if not link.exists():
                    link.symlink_to(library)
    print("Build complete. Start a fresh pixi shell from ws, then run pixi run doctor.")


if __name__ == "__main__":
    main()
