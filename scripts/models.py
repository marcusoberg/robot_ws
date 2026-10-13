"""Run optional model commands with their own interpreter and no ROS overlay."""

from pathlib import Path
import argparse
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command", choices=("setup", "serve", "train", "inspect", "export")
    )
    args, extra = parser.parse_known_args()
    env = dict(os.environ)
    for name in (
        "PYTHONPATH",
        "PYTHONHOME",
        "LD_LIBRARY_PATH",
        "DYLD_LIBRARY_PATH",
        "DYLD_FALLBACK_LIBRARY_PATH",
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
    ):
        env.pop(name, None)
    if "--offline" in extra:
        env["HF_HUB_OFFLINE"] = "1"
        env["TRANSFORMERS_OFFLINE"] = "1"
        # An extracted SmolVLA bundle carries its own portable offline resources.
        checkpoint = None
        for flag in ("--checkpoint", "--resume"):
            if flag in extra and extra.index(flag) + 1 < len(extra):
                checkpoint = Path(extra[extra.index(flag) + 1]).resolve()
                break
        bundled = checkpoint.parent / "huggingface" if checkpoint else None
        env.setdefault(
            "HF_HUB_CACHE",
            str(bundled if bundled and bundled.is_dir()
                else ROOT / "artifacts/legacy-20261011/huggingface"),
        )
    project = ROOT / "environments/models"
    if args.command == "setup":
        command = [
            "uv",
            "sync",
            "--locked",
            "--python",
            sys.executable,
            "--project",
            str(project),
            *extra,
        ]
    else:
        python = project / ".venv/bin/python"
        if not python.exists():
            parser.error("Run pixi run setup-models first.")
        module = {
            "serve": "model_server.serving.server",
            "train": "model_training.prepare",
            "inspect": "model_server.inspect",
            "export": "recording_tools.cli",
        }[args.command]
        command = [str(python), "-m", module, *extra]
    return subprocess.call(command, cwd=ROOT, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
