from pathlib import Path

from robot_core.config import read_settings


def teleop_settings():
    return read_settings(Path(__file__).with_name("teleop.toml"))


def tracking_profile(name):
    config = teleop_settings()
    profile = config["teleop"][name]
    return profile | {"leader": config["leader"] | {"tip": profile["leader_tip"]},
                      "mapping": config["mapping"]}
