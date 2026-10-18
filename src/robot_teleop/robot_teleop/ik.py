"""URDF-based FK and bounded pose IK for the teleoperation application."""

from dataclasses import dataclass
import json
from pathlib import Path
import tomllib
import xml.etree.ElementTree as ET

import numpy as np
from robot_description.spec import ActionSpec


def model_path(value, config_path):
    prefix = 'package://robot_description/'
    if value.startswith(prefix):
        import robot_description
        path = Path(robot_description.__file__).parent / 'assets' / value[len(prefix):]
    else:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path(config_path).resolve().parent / path
    return path.resolve(strict=True)


class UrdfModel:
    def __init__(self, description, joints, base, tip, **observation):
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise ImportError('FK/IK requires robot-teleop[ik]; use the workspace ROS environment') from exc
        self.pin = pin
        self.description = Path(description)
        self.joints = tuple(joints)

        root = ET.parse(description).getroot()
        defined = {j.attrib['name']: j for j in root.findall('joint') if j.attrib['type'] != 'fixed'}

        if len(set(self.joints)) != len(self.joints) or set(defined) != set(self.joints):
            raise ValueError('Configured joints must name every movable URDF joint exactly once')

        lower, upper, units = [], [], []
        for name in self.joints:
            joint = defined[name]

            if joint.attrib['type'] not in ('revolute', 'prismatic') or joint.find('mimic') is not None:
                raise ValueError(f'IK supports independent bounded revolute/prismatic joints: {name}')

            limit = joint.find('limit')

            if limit is None or not {'lower', 'upper'} <= limit.attrib.keys():
                raise ValueError(f'URDF joint requires position bounds: {name}')

            lower.append(float(limit.attrib['lower']))
            upper.append(float(limit.attrib['upper']))
            units.append('rad' if joint.attrib['type'] == 'revolute' else 'm')

        self.spec = ActionSpec(self.joints, tuple(units), tuple(lower), tuple(upper), **observation)
        self.model = pin.buildModelFromUrdf(str(description))
        self.data = self.model.createData()
        self.indices = []
        for name in self.joints:
            if not self.model.existJointName(name):
                raise ValueError(f'URDF joint missing from kinematic model: {name}')
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f'IK requires scalar joints: {name}')
            self.indices.append(joint.idx_q)
        for frame in (base, tip):
            if not self.model.existFrame(frame):
                raise ValueError(f'Unknown URDF frame: {frame}')
        self.base = self.model.getFrameId(base)
        self.tip = self.model.getFrameId(tip)
        if self.model.frames[self.base].parentJoint != 0:
            raise ValueError('The configured base must be fixed to the URDF root')
        chain = set()
        joint_id = self.model.frames[self.tip].parentJoint
        while joint_id:
            chain.add(self.model.names[joint_id])
            joint_id = self.model.parents[joint_id]
        self.active = np.array([i for i, name in enumerate(self.joints) if name in chain], dtype=int)
        if not len(self.active):
            raise ValueError('No movable joints between the configured base and tip')

    def positions(self, positions):
        values = np.asarray(positions, dtype=float)
        if values.shape != (len(self.joints),) or not np.isfinite(values).all():
            raise ValueError('Joint state requires one finite position per configured joint')
        return values.copy()

    def pose(self, positions):
        values = self.positions(positions)
        q = self.pin.neutral(self.model)
        q[self.indices] = values
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.base].inverse() * self.data.oMf[self.tip]


@dataclass(frozen=True)
class SolverSettings:
    position_scale: float = 0.001
    orientation_scale: float = 0.01
    orientation: bool = True
    max_evaluations: int = 80


class PoseIK:
    def __init__(self, model, settings):
        from scipy.optimize import least_squares
        self.optimize = least_squares
        self.model = model
        self.settings = settings

    def solve(self, target, seed):
        q = self.model.positions(seed)
        active = self.model.active
        lower = np.asarray(self.model.spec.lower)[active]
        upper = np.asarray(self.model.spec.upper)[active]
        settings = self.settings
        def residual(values):
            q[active] = values
            pose = self.model.pose(q)
            position = (pose.translation - target.translation) / settings.position_scale
            if settings.orientation:
                orientation = self.model.pin.log3(target.rotation @ pose.rotation.T) / settings.orientation_scale
                return np.concatenate((position, orientation))
            return position

        # A measured state may cross a limit. Project only the optimizer's starting
        # iterate into its feasible domain; the measured state and target are unchanged.
        initial = np.clip(q[active], lower, upper)
        result = self.optimize(residual, initial, method='trf', bounds=(lower, upper),
                               max_nfev=settings.max_evaluations, ftol=1e-9, xtol=1e-9, gtol=1e-9)
        q[active] = result.x
        return self.model.spec.validate(q)


class FKIKMapping:
    def __init__(self, config_path):
        config = tomllib.loads(Path(config_path).read_text())
        if config.get('version') != 1:
            raise ValueError('Unsupported FK/IK configuration version')
        leader, follower = config['leader'], config['follower']
        self.leader = UrdfModel(model_path(leader['urdf'], config_path), leader['joints'], leader['base'], leader['tip'])
        self.follower = UrdfModel(model_path(follower['urdf'], config_path), follower['joints'],
                                  follower['base'], follower['tip'], **config.get('observation', {}))
        self.spec = self.follower.spec
        self.leader_reference = self.leader.spec.validate(leader['reference'])
        self.reference = self.spec.validate(follower['reference'])
        self.leader_pose = self.leader.pose(self.leader_reference)
        self.follower_pose = self.follower.pose(self.reference)
        mapping = config.get('mapping', {})
        self.scale = float(mapping.get('translation_scale', 1.))
        rpy = np.asarray(mapping.get('rotation_rpy', [0., 0., 0.]), dtype=float)
        if not np.isfinite(self.scale) or self.scale <= 0 or rpy.shape != (3,) or not np.isfinite(rpy).all():
            raise ValueError('Invalid leader-to-follower frame mapping')
        self.rotation = self.leader.pin.rpy.rpyToMatrix(rpy)
        self.ticks_per_revolution = float(leader.get('ticks_per_revolution', 4096))
        self.direction = np.asarray(leader.get('direction', [1.] * len(self.leader.joints)), dtype=float)
        self.zero = np.asarray(leader.get('zero_radians', [0.] * len(self.leader.joints)), dtype=float)
        if (not np.isfinite(self.ticks_per_revolution) or self.ticks_per_revolution <= 0
                or self.direction.shape != (len(self.leader.joints),) or not np.isin(self.direction, [-1, 1]).all()
                or self.zero.shape != self.direction.shape or not np.isfinite(self.zero).all()
                or any(unit != 'rad' for unit in self.leader.spec.units)):
            raise ValueError('Invalid rotary leader encoder conversion')
        self.direct = config.get('direct', [])
        output_names = []
        for binding in self.direct:
            if binding['input'] not in self.leader.joints or binding['joint'] not in self.spec.joints:
                raise ValueError('Unknown joint in direct mapping')
            index = self.spec.joints.index(binding['joint'])
            values = (binding['closed'], binding['open'])
            if (index in self.follower.active or not np.isfinite(values).all() or values[0] == values[1]
                    or min(values) < self.spec.lower[index] or max(values) > self.spec.upper[index]):
                raise ValueError('Direct mapping must control a joint outside the IK chain, within its URDF bounds')
            output_names.append(binding['joint'])
        passive = {name for i, name in enumerate(self.spec.joints) if i not in self.follower.active}
        if len(set(output_names)) != len(output_names) or set(output_names) != passive:
            raise ValueError('Each commanded joint outside the IK chain requires exactly one direct mapping')
        self.solver = PoseIK(self.follower, SolverSettings(**config.get('solver', {})))

    def target_pose(self, positions):
        pose = self.leader.pose(positions)
        translation = self.follower_pose.translation + self.scale * self.rotation @ (pose.translation - self.leader_pose.translation)
        delta = self.rotation @ (pose.rotation @ self.leader_pose.rotation.T) @ self.rotation.T
        return self.leader.pin.SE3(delta @ self.follower_pose.rotation, translation)

    def solve(self, positions, seed=None, direct=None):
        seed = self.follower.positions(self.reference if seed is None else seed)
        for joint, value in (direct or {}).items():
            if joint not in {binding['joint'] for binding in self.direct}:
                raise ValueError(f'Joint is not configured for direct control: {joint}')
            seed[self.spec.joints.index(joint)] = value
        return self.solver.solve(self.target_pose(positions), seed)

    def map(self, raw, calibration, seed=None):
        if set(raw) != set(self.leader.joints):
            raise ValueError('Leader sample joint names differ from IK input model')
        positions, fractions = [], {}
        for i, name in enumerate(self.leader.joints):
            c = calibration[name]
            low, high = c['range_min'], c['range_max']
            value = raw[name]
            if not np.isfinite(value) or not 0 <= value < self.ticks_per_revolution:
                raise ValueError(f'Leader {name} sample {value} outside encoder domain [0, {self.ticks_per_revolution:g})')
            neutral = c.get('neutral', (low + high) / 2)
            positions.append(self.zero[i] + self.direction[i] * (value - neutral) * 2 * np.pi / self.ticks_per_revolution)
            fractions[name] = (value - low) / (high - low)
        direct = {}
        for binding in self.direct:
            fraction = fractions[binding['input']]
            if not 0 <= fraction <= 1:
                name = binding['input']
                c = calibration[name]
                raise ValueError(f'Leader {name} sample {raw[name]} outside calibrated '
                                 f'[{c["range_min"]}, {c["range_max"]}] for direct joint mapping')
            direct[binding['joint']] = (binding['closed'] if fraction == 0 else binding['open'] if fraction == 1
                                       else binding['closed'] + fraction * (binding['open'] - binding['closed']))
        return self.solve(positions, seed, direct)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Solve leader FK → follower IK without ROS or a connected robot')
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--leader-positions', required=True, help='JSON array of leader joint angles in configured order, radians')
    parser.add_argument('--seed', help='JSON array of follower joint positions; defaults to the configured reference pose')
    args = parser.parse_args()
    mapper = FKIKMapping(args.config)
    goal = mapper.solve(json.loads(args.leader_positions), None if args.seed is None else json.loads(args.seed))
    print(json.dumps(dict(joints=mapper.spec.joints, positions=goal.tolist(), action_spec_id=mapper.spec.identity)))


if __name__ == '__main__':
    main()
