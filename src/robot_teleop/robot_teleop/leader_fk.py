"""SO101 forward kinematics and frame mapping only. Robot IK stays in the SDK."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

def transform(value):
    xyz = np.asarray(value.get("xyz", [0, 0, 0]), dtype=float)
    rpy = np.asarray(value.get("rpy", [0, 0, 0]), dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
        raise ValueError("Transform needs finite xyz metres and rpy radians")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result

class LeaderFK:
    def __init__(self, urdf, config):
        self.config = config
        parents = {j.find("child").get("link"): j for j in ET.parse(urdf).getroot().findall("joint")}
        self.chain = []
        tip = config["tip"]
        while tip != config["base"]:
            if tip not in parents or len(self.chain) > len(parents):
                raise ValueError("Leader base/tip do not form a URDF chain")
            j = parents[tip]
            if j.get("type") not in ("fixed", "revolute", "continuous"):
                raise ValueError("Unsupported leader joint type")
            self.chain.insert(0, j)
            tip = j.find("parent").get("link")
        if {j.get("name") for j in self.chain if j.get("type") != "fixed"} != set(JOINTS):
            raise ValueError("Leader handle chain must contain exactly the five arm joints")
        self.sign = np.asarray(config["joint_sign"], dtype=float)
        self.offset = np.deg2rad(config["joint_offset_degrees"])
        self.scale = float(config.get("position_scale", 1))
        if self.sign.shape != (5,) or self.offset.shape != (5,) or not np.isfinite([self.sign,self.offset]).all() or not np.all(np.abs(self.sign)==1) or not 0 < self.scale <= 10:
            raise ValueError("Invalid leader joint calibration or position scale")
        self.mount = transform(config["robot_from_leader"])
        self.tool = transform(config["handle_to_tool"])

    def angles(self, degrees):
        q = self.sign * np.deg2rad(degrees) + self.offset
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError("Invalid leader angles")
        return dict(zip(JOINTS, q))

    def pose(self, degrees):
        angles = self.angles(degrees)
        result = np.eye(4)
        for j in self.chain:
            origin = j.find("origin")
            attrs = {} if origin is None else {k: [float(x) for x in v.split()] for k,v in origin.attrib.items()}
            result = result @ transform(attrs)
            if j.get("type") != "fixed":
                axis = np.asarray([float(x) for x in j.find("axis").get("xyz").split()])
                if np.linalg.norm(axis) < 1e-9:
                    raise ValueError("Invalid URDF joint axis")
                rotation = np.eye(4)
                rotation[:3,:3] = Rotation.from_rotvec(axis/np.linalg.norm(axis)*angles[j.get("name")]).as_matrix()
                result = result @ rotation
        result[:3,3] *= self.scale
        return self.mount @ result @ self.tool
