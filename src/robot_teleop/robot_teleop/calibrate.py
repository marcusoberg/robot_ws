"""Run the interactive LeRobot calibrator with this leader's configured identity."""

import os
from pathlib import Path

from robot_teleop.config import teleop_settings


def main():
    hardware = teleop_settings()["hardware"]
    python = Path(os.environ["PIXI_PROJECT_ROOT"]) / "environments/models/.venv/bin/python"
    os.execv(python, [str(python), "-m", "lerobot.scripts.lerobot_calibrate",
                      "--teleop.type=so101_leader", f"--teleop.port={hardware['leader_port']}",
                      f"--teleop.id={hardware['leader_id']}",
                      f"--teleop.calibration_dir={Path(hardware['calibration_dir']).expanduser()}"])


if __name__ == "__main__":
    main()
