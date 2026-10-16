"""Explicit interactive LeRobot calibration; never performed by the launch file."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--leader-id", default="so101-leader")
    p.add_argument("--calibration-dir")
    p.add_argument("--python")
    a = p.parse_args()
    root = os.environ.get("PIXI_PROJECT_ROOT")
    if not a.python and not root:
        p.error("Use the workspace Pixi shell or supply --python for the LeRobot environment")
    python = a.python or str(Path(root)/"environments/models/.venv/bin/python")
    env = dict(os.environ)
    for key in ("PYTHONPATH","PYTHONHOME","LD_LIBRARY_PATH","DYLD_LIBRARY_PATH","DYLD_FALLBACK_LIBRARY_PATH"):
        env.pop(key, None)
    env["PYTHONPATH"] = str(Path(__file__).resolve().parent.parent)
    command = [python,"-m","robot_teleop.lerobot_worker","leader","--calibrate",
               "--port",a.port,"--leader-id",a.leader_id]
    if a.calibration_dir:
        command += ["--calibration-dir",a.calibration_dir]
    raise SystemExit(subprocess.call(command, env=env))
