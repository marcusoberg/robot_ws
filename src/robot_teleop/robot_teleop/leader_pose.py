"""SO101 encoder conversion, leader FK and relative TCP mapping."""

from pathlib import Path

import numpy as np
import pinocchio as pin

import robot_description
from robot_core.data.spec import ActionSpec
from robot_core.kinematics.chain import forward_kinematics, load_chain


class LeaderPose:
    def __init__(self, config):
        leader = config['leader']
        path = Path(robot_description.__file__).parent / 'assets' / leader['urdf'].removeprefix('package://robot_description/')
        self.chain = load_chain(path, leader['joints'], leader['base'], leader['tip'])
        self.data = self.chain.model.createData()
        self.spec = ActionSpec.from_dict(dict(joints=self.chain.joints, units=self.chain.units,
                                             lower=self.chain.lower, upper=self.chain.upper))
        self.ticks_per_revolution = leader['ticks_per_revolution']
        self.direction = np.asarray(leader['direction'])
        self.zero = np.asarray(leader['zero_radians'])
        self.scale = config['mapping']['translation_scale']
        self.frame_rotation = pin.rpy.rpyToMatrix(np.asarray(config['mapping']['rotation_rpy']))
        self.direct = config['direct']

    def read(self, raw, calibration):
        names = self.chain.joints
        ticks = np.array([raw[name] for name in names])
        low = np.array([calibration[name]['range_min'] for name in names])
        high = np.array([calibration[name]['range_max'] for name in names])
        angles = self.zero + self.direction * (ticks - (low + high) / 2) * (2 * np.pi / self.ticks_per_revolution)
        grip = []
        for binding in self.direct:
            i = names.index(binding['input'])
            fraction = (ticks[i] - low[i]) / (high[i] - low[i])
            grip.append(float(binding['closed'] + fraction * (binding['open'] - binding['closed'])))
        return forward_kinematics(self.chain, angles, self.data), grip

    def align(self, leader, tcp):
        self.leader_origin = leader.copy()
        self.tcp_origin = tcp.copy()
        self.rotation = self.frame_rotation @ tcp.rotation @ leader.rotation.T

    def target(self, leader):
        xyz = self.tcp_origin.translation + self.scale * self.rotation @ (leader.translation - self.leader_origin.translation)
        delta = self.rotation @ (leader.rotation @ self.leader_origin.rotation.T) @ self.rotation.T
        return pin.SE3(delta @ self.tcp_origin.rotation, xyz)
