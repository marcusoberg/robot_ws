"""Offline Cartesian geometry preview. No ROS nodes, simulator process or serial connection."""

import argparse
from robot_core.config import robot_settings
from robot_core.kinematics.chain import load_chain, forward_kinematics
from robot_description.spec import parol6
from robot_path.planner import PathPlanner
from robot_path.settings import PathSettings, path_settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=float, default=path_settings()["path"]["width"])
    parser.add_argument("--distance", type=float, default=path_settings()["path"]["distance"])
    parser.add_argument("--height", type=float, default=path_settings()["path"]["height"])
    parser.add_argument("--speed", type=float, default=path_settings()["path"]["speed"])
    parser.add_argument(
        "--gripper", choices=["pneumatic", "msg_ai_100"], default=path_settings()["path"]["gripper"]
    )
    parser.add_argument("--tip", default=path_settings()["path"]["tip"])
    settings = PathSettings.configured(**vars(parser.parse_args()))
    profile = parol6(settings.gripper, scene="robot_only")
    chain = load_chain(profile.description, profile.spec.joints, robot_settings()['cartesian']['base'], settings.tip)
    start = forward_kinematics(chain, profile.initial, chain.model.createData())
    plan = PathPlanner(settings).plan(start.translation)
    print(plan.summary())
    print(
        "Cartesian extent in base_link [m]:",
        plan.xyz.min(axis=0),
        "to",
        plan.xyz.max(axis=0),
    )


if __name__ == "__main__":
    main()
