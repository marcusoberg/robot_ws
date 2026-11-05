"""SO101 encoder conversion and application-specific leader-to-follower mapping."""

from pathlib import Path
import tomllib

import numpy as np
import pinocchio as pin

import robot_description
from robot_description.spec import ActionSpec
from robot_kinematics import IKSettings, PoseIK, UrdfModel


def model_path(value, config_path):
    prefix = "package://robot_description/"
    if value.startswith(prefix):
        return Path(robot_description.__file__).parent / "assets" / value[len(prefix):]
    return (Path(config_path).resolve().parent / Path(value).expanduser()).resolve()


class FKIKMapping:
    def __init__(self, config_path):
        config = tomllib.loads(Path(config_path).read_text())
        leader, follower = config["leader"], config["follower"]
        self.leader = UrdfModel(model_path(leader["urdf"], config_path),
                                leader["joints"], leader["base"], leader["tip"])
        self.follower = UrdfModel(model_path(follower["urdf"], config_path),
                                  follower["joints"], follower["base"], follower["tip"])
        self.spec = ActionSpec(self.follower.joints, self.follower.units,
                               self.follower.lower, self.follower.upper, **config["observation"])
        self.leader_spec = ActionSpec(self.leader.joints, self.leader.units,
                                      self.leader.lower, self.leader.upper)
        self.leader_reference = self.leader.validate(leader["reference"])
        self.reference = self.follower.validate(follower["reference"])
        self.startup_duration = follower["startup_duration"]
        self.leader_pose = self.leader.pose(self.leader_reference)
        self.follower_pose = self.follower.pose(self.reference)
        self.scale = config["mapping"]["translation_scale"]
        self.frame_rotation = pin.rpy.rpyToMatrix(np.asarray(config["mapping"]["rotation_rpy"]))
        self.rotation = self.frame_rotation @ self.follower_pose.rotation @ self.leader_pose.rotation.T
        self.ticks_per_revolution = leader["ticks_per_revolution"]
        self.direction = np.asarray(leader["direction"])
        self.zero = np.asarray(leader["zero_radians"])
        self.direct = config["direct"]
        self.solver = PoseIK(self.follower, IKSettings(**config["solver"]))

    def target_pose(self, positions):
        pose = self.leader.pose(positions)
        translation = self.follower_pose.translation + self.scale * self.rotation @ (
            pose.translation - self.leader_pose.translation
        )
        delta = self.rotation @ (pose.rotation @ self.leader_pose.rotation.T) @ self.rotation.T
        return pin.SE3(delta @ self.follower_pose.rotation, translation)

    def solve(self, positions, seed, *, continuity=None):
        return self.solver.solve(self.target_pose(positions), seed, continuity=continuity)

    def align_leader(self, raw, calibration):
        """Align the current leader TCP with the follower's configured reference TCP."""
        self.leader_reference, _ = self.decode(raw, calibration)
        self.leader_pose = self.leader.pose(self.leader_reference)
        self.rotation = self.frame_rotation @ self.follower_pose.rotation @ self.leader_pose.rotation.T

    def map(self, raw, calibration, seed, *, continuity=None):
        positions, direct = self.decode(raw, calibration)
        seed = np.asarray(seed, dtype=float).copy()
        for joint, value in direct.items():
            seed[self.spec.joints.index(joint)] = value
        return self.solve(positions, seed, continuity=continuity)

    def decode(self, raw, calibration):
        ticks = np.asarray([raw[name] for name in self.leader.joints])
        low = np.asarray([calibration[name]["range_min"] for name in self.leader.joints])
        high = np.asarray([calibration[name]["range_max"] for name in self.leader.joints])
        neutral = (low + high) / 2
        positions = self.zero + self.direction * (ticks - neutral) * (2 * np.pi / self.ticks_per_revolution)
        fractions = dict(zip(self.leader.joints, (ticks - low) / (high - low)))
        direct = {}
        for binding in self.direct:
            fraction = fractions[binding["input"]]
            direct[binding["joint"]] = (1 - fraction) * binding["closed"] + fraction * binding["open"]
        return positions, direct
