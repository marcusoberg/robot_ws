"""Read the configuration owned by each package once per process."""

from functools import cache
from pathlib import Path
import tomllib


@cache
def read_settings(path):
    with path.open("rb") as stream:
        return tomllib.load(stream)


def robot_settings():
    return read_settings(Path(__file__).with_name("robot.toml"))
