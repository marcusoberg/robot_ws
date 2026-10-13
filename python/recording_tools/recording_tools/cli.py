"""Inspect alignment or export a completed recording using explicit camera mapping."""

import argparse
import json
import math
from pathlib import Path
from .alignment import align, export_lerobot


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", required=True, type=Path)
    p.add_argument("--output", type=Path)
    p.add_argument(
        "--config",
        "--features",
        dest="config",
        required=True,
        type=Path,
        help="JSON/YAML with joints, cameras(feature to source), and period",
    )
    p.add_argument(
        "--alignment-only",
        action="store_true",
        help="Inspect approved aligned segments without LeRobot or writing a dataset",
    )
    p.add_argument("--accuracy-budget-ns", type=int, default=1000000)
    a = p.parse_args()
    if a.config.suffix == ".json":
        config = json.loads(a.config.read_text())
    else:
        import yaml

        config = yaml.safe_load(a.config.read_text())
    if not isinstance(config, dict):
        p.error("Configuration must be a mapping")
    cameras = config.get("cameras")
    joints = config.get("joints", config.get("state_joints"))
    period = config.get("period", config.get("action_period"))
    if (
        not isinstance(cameras, dict)
        or any(
            not isinstance(k, str) or not isinstance(v, str) for k, v in cameras.items()
        )
        or not cameras
        or not isinstance(joints, (list, tuple))
        or not joints
        or any(not isinstance(name, str) or not name for name in joints)
        or len(set(joints)) != len(joints)
        or type(period) not in (float, int)
        or not math.isfinite(period)
        or period <= 0
    ):
        p.error(
            "Supply joints, feature-to-source camera names and a positive action period"
        )
    budget = config.get("accuracy_budget_ns", a.accuracy_budget_ns)
    if type(budget) is not int or budget < 0:
        p.error("Alignment accuracy budget must be a nonnegative integer")
    if a.alignment_only:
        episodes, gaps = align(
            a.run,
            period=period,
            cameras=cameras,
            accuracy_budget_ns=budget,
            include_images=False,
        )
        manifest = json.loads((a.run / "manifest.json").read_text())
        result = {
            "complete": manifest.get("complete", False),
            "approved_segments": len(episodes),
            "aligned_frames": sum(len(rows) for _, rows in episodes.values()),
            "gaps": gaps,
            "period": period,
        }
    else:
        if a.output is None:
            p.error("Export requires --output")
        result = export_lerobot(
            a.run,
            a.output,
            period=period,
            cameras=cameras,
            joints=joints,
            accuracy_budget_ns=budget,
        )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
