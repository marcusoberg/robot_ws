"""Build robot profiles from workspace settings and the model's joint limits."""

from pathlib import Path
import xml.etree.ElementTree as ET

from robot_core.config import robot_settings
from robot_core.data.spec import ActionSpec, Profile


def _profile(robot, description, scene_file, initial, cameras, **overrides):
    assets = Path(__file__).parent / "assets" / robot["assets"]
    urdf = assets / description
    joints = {joint.attrib["name"]: joint for joint in ET.parse(urdf).getroot().findall("joint")}
    ordered = [joints[name] for name in robot["joints"]]
    limits = [joint.find("limit").attrib for joint in ordered]
    spec = ActionSpec.from_dict(dict(
        joints=robot["joints"], units=["m" if j.attrib["type"] == "prismatic" else "rad" for j in ordered],
        lower=[float(limit["lower"]) for limit in limits], upper=[float(limit["upper"]) for limit in limits],
        cameras=cameras,
    ))
    return Profile(spec, assets / scene_file, urdf, tuple(initial), **(robot_settings()["profile"] | overrides))


def so101():
    robot = robot_settings()["robots"]["so101"]
    return _profile(robot, robot["description"], robot["scene"], robot["initial"], robot_settings()["action"]["cameras"])


def parol6(gripper=None, *, scene=None):
    robot = robot_settings()["robots"]["parol6"]
    tool = robot[robot["gripper"] if gripper is None else gripper]
    scene = robot_settings()["profile"]["task"] if scene is None else scene
    cameras = () if scene == "robot_only" else robot_settings()["action"]["cameras"]
    return _profile(robot, tool["description"], tool[scene], [*robot["arm_initial"], tool["gripper_initial"]],
                    cameras, cube_half_size=robot["cube_half_size"], task=scene)


def load_profile(robot="so101", *, gripper=None, scene=None):
    if robot == "parol6":
        return parol6(gripper, scene=scene)
    return {"so101": so101}[robot]()
