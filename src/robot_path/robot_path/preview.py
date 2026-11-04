"""Offline IK inspection. No ROS nodes, simulator process or serial connection."""

import argparse
from robot_description.spec import parol6
from robot_path.planner import PathPlanner
from robot_path.settings import PathSettings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=float, default=0.18)
    parser.add_argument("--distance", type=float, default=0.30)
    parser.add_argument("--height", type=float, default=0.16)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--gripper", choices=["pneumatic", "msg_ai_100"], default="pneumatic"
    )
    parser.add_argument("--tip", default="tool_tcp")
    settings = PathSettings(**vars(parser.parse_args()))
    profile = parol6(settings.gripper, scene="robot_only")
    plan = PathPlanner(settings).plan(profile.initial, progress=print)
    print(plan.summary())
    print(
        "Cartesian extent in base_link [m]:",
        plan.xyz.min(axis=0),
        "to",
        plan.xyz.max(axis=0),
    )


if __name__ == "__main__":
    main()
