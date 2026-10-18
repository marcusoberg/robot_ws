from pathlib import Path

import numpy as np
import pytest

pin = pytest.importorskip('pinocchio', reason='FK/IK is an optional teleop dependency')
pytest.importorskip('scipy')

from robot_description.spec import so101
from robot_teleop.ik import FKIKMapping, PoseIK, SolverSettings, UrdfModel


CONFIG = Path(__file__).resolve().parents[1] / 'src/robot_teleop/config'


def mapping(name='so101'):
    return FKIKMapping(CONFIG / f'{name}_ik.toml')


def assert_pose(actual, expected, position=1e-6, orientation=1e-6):
    assert np.linalg.norm(actual.translation - expected.translation) < position
    assert np.linalg.norm(pin.log3(expected.rotation @ actual.rotation.T)) < orientation


def chain_urdf(path, count):
    links = '<link name="world"/><link name="base"/>'
    joints = '''<joint name="mount" type="fixed"><parent link="world"/><child link="base"/>
      <origin xyz="1 2 3" rpy="0 0 1"/></joint>'''
    for i in range(count):
        links += f'<link name="link{i}"/>'
        parent = 'base' if i == 0 else f'link{i-1}'
        length = 0 if i == 0 else .3 if i == 1 else .2
        joints += f'''<joint name="j{i}" type="revolute"><parent link="{parent}"/><child link="link{i}"/>
          <origin xyz="{length} 0 0"/><axis xyz="0 0 1"/>
          <limit lower="-2" upper="2" effort="10" velocity="2"/></joint>'''
    links += '<link name="tool"/>'
    joints += f'''<joint name="tcp" type="fixed"><parent link="link{count-1}"/><child link="tool"/>
      <origin xyz="0.1 0 0"/></joint>'''
    path.write_text(f'<robot name="chain">{links}{joints}</robot>')
    return path


@pytest.mark.parametrize('count', [3, 7])
def test_generic_fk_and_ik_use_urdf_chain_and_declared_order(tmp_path, count):
    path = chain_urdf(tmp_path / 'chain.urdf', count)
    order = tuple(f'j{i}' for i in reversed(range(count)))
    model = UrdfModel(path, order, 'base', 'tool')
    natural_q = np.linspace(.1, .3, count)
    q = natural_q[::-1].copy()
    angles = np.cumsum(natural_q)
    lengths = np.array([.3] + [.2] * (count - 2) + [.1])
    expected_position = [np.dot(lengths, np.cos(angles)), np.dot(lengths, np.sin(angles)), 0.]
    expected = pin.SE3(pin.rpy.rpyToMatrix(0., 0., float(sum(natural_q))), np.array(expected_position))
    assert_pose(model.pose(q), expected)
    result = PoseIK(model, SolverSettings()).solve(expected, q + .02)
    model.spec.validate(result)
    assert_pose(model.pose(result), expected)


def test_prismatic_joint_uses_metres(tmp_path):
    path = tmp_path / 'slider.urdf'
    path.write_text('''<robot name="slider"><link name="base"/><link name="tool"/>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>
        <origin xyz="0 0.1 0"/><axis xyz="1 0 0"/>
        <limit lower="-0.2" upper="0.2" effort="10" velocity="1"/>
      </joint></robot>''')
    model = UrdfModel(path, ['slide'], 'base', 'tool')
    assert model.spec.units == ('m',)
    target = pin.SE3(np.eye(3), np.array([.12, .1, 0]))
    goal = PoseIK(model, SolverSettings()).solve(target, [0.])
    assert goal == pytest.approx([.12])


@pytest.mark.parametrize('joints,base,tip,message', [
    (['j0', 'j1'], 'base', 'tool', 'every movable'),
    (['j0', 'j0', 'j2'], 'base', 'tool', 'every movable'),
    (['j0', 'j1', 'j2'], 'absent', 'tool', 'Unknown URDF frame'),
    (['j0', 'j1', 'j2'], 'base', 'absent', 'Unknown URDF frame'),
    (['j0', 'j1', 'j2'], 'link0', 'tool', 'base must be fixed'),
])
def test_rejects_incompatible_urdf_configuration(tmp_path, joints, base, tip, message):
    path = chain_urdf(tmp_path / 'chain.urdf', 3)
    with pytest.raises(ValueError, match=message):
        UrdfModel(path, joints, base, tip)


@pytest.mark.parametrize('name', ['so101', 'parol6'])
@pytest.mark.parametrize('leader_q', [
    [0.] * 6,
    [.1, -.05, .1, .05, .02, 0.],
    [.2, -.1, .2, .1, .05, 0.],
])
def test_follower_changes_by_configuration_only(name, leader_q):
    mapper = mapping(name)
    goal = mapper.solve(leader_q)
    mapper.spec.validate(goal)
    assert_pose(mapper.follower.pose(goal), mapper.target_pose(leader_q))
    if name == 'so101':
        so101().spec.require(mapper.spec)
        assert mapper.follower.active.tolist() == list(range(5))
    else:
        assert mapper.spec.joints == ('L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'gripper_slide')
        assert mapper.follower.active.tolist() == list(range(6))


@pytest.mark.parametrize('name', ['parol6', 'parol6_msg_ai_100'])
def test_leader_wrist_twist_rotates_follower_wrist_without_branch_switch(name):
    from robot_description.spec import parol6
    from robot_server_core.simulation import SimulationEnv
    mapper = mapping(name)
    profile = parol6('msg_ai_100' if name == 'parol6_msg_ai_100' else 'pneumatic')
    env = SimulationEnv(profile, cameras=False)
    angles = np.r_[np.arange(0, 171, 10), np.arange(170, -171, -10), np.arange(-170, 1, 10)]
    try:
        env.activate('wrist-regression')
        observation = env.observation()
        previous = mapper.reference.copy()
        for degrees in angles:
            leader = mapper.leader_reference.copy()
            leader[mapper.leader.joints.index('wrist_roll')] = np.deg2rad(degrees)
            goal = mapper.solve(leader, observation.position, {mapper.spec.joints[-1]: mapper.reference[-1]})
            np.testing.assert_allclose(goal[:5], mapper.reference[:5], atol=1e-6)
            assert goal[5] == pytest.approx(mapper.reference[5] + np.deg2rad(degrees), abs=1e-6)
            assert np.max(np.abs(goal - previous)) <= np.deg2rad(10) + 1e-6
            for _ in range(3):
                observation, _ = env.step(goal)
            assert abs(observation.position[5] - goal[5]) < np.deg2rad(1)
            previous = goal
    finally:
        env.close()


def test_encoder_angles_are_not_rescaled_to_follower_limits_and_gripper_is_separate():
    mapper = mapping()
    calibration = {name: dict(id=i+1, drive_mode=0, homing_offset=0, range_min=100, range_max=3900)
                   for i, name in enumerate(mapper.leader.joints)}
    raw = dict(zip(mapper.leader.joints, [2030, 1980, 2040, 2020, 2050, 100]))
    physical_angles = [(raw[name] - 2000) * 2 * np.pi / 4096 for name in mapper.leader.joints]
    expected = mapper.target_pose(physical_angles)
    for grip, expected_grip in ((100, mapper.spec.lower[-1]), (2000, (mapper.spec.lower[-1] + mapper.spec.upper[-1])/2),
                                (3900, mapper.spec.upper[-1])):
        raw['gripper'] = grip
        goal = mapper.map(raw, calibration)
        assert goal[-1] == pytest.approx(expected_grip)
        assert_pose(mapper.follower.pose(goal), expected)
    raw['elbow_flex'] = 4096
    with pytest.raises(ValueError, match='outside encoder domain'):
        mapper.map(raw, calibration)
    raw['elbow_flex'] = float('nan')
    with pytest.raises(ValueError, match='outside encoder domain'):
        mapper.map(raw, calibration)


def test_ik_encoder_conversion_preserves_angles_past_calibration_extrema():
    mapper = mapping()
    calibration = {name: dict(id=i+1, drive_mode=0, homing_offset=0, range_min=100, range_max=3900)
                   for i, name in enumerate(mapper.leader.joints)}
    calibration['elbow_flex'].update(range_min=857, range_max=3063)
    raw = dict.fromkeys(mapper.leader.joints, 2000)
    previous_angle = None
    for value in (3062, 3063, 3064, 3065):
        raw['elbow_flex'] = value
        angles = [(raw[name] - (c['range_min'] + c['range_max'])/2) * 2*np.pi/4096
                  for name in mapper.leader.joints for c in [calibration[name]]]
        goal = mapper.map(raw, calibration)
        mapper.spec.validate(goal)
        assert_pose(mapper.follower.pose(goal), mapper.target_pose(angles), .001, .01)
        if previous_angle is not None:
            assert angles[2] - previous_angle == pytest.approx(2*np.pi/4096)
        previous_angle = angles[2]
    raw['gripper'] = 3901
    with pytest.raises(ValueError, match='outside calibrated.*direct joint mapping'):
        mapper.map(raw, calibration)


@pytest.mark.parametrize('index,side', [(0, -1), (2, 1)])
def test_measured_arm_state_outside_bounds_is_an_ik_seed_not_a_command(index, side):
    mapper = mapping()
    seed = mapper.reference.copy()
    boundary = mapper.spec.lower[index] if side < 0 else mapper.spec.upper[index]
    seed[index] = boundary + side * .01
    seed[-1] = mapper.spec.lower[-1] - .001
    original = seed.copy()
    leader_q = [0.] * 6
    leader_q[index] = boundary - side * .1
    goal = mapper.solve(leader_q, seed, {'gripper': mapper.spec.lower[-1]})
    np.testing.assert_array_equal(seed, original)
    mapper.spec.validate(goal)
    assert abs(goal[index] - boundary) > .05
    assert_pose(mapper.follower.pose(goal), mapper.target_pose(leader_q))


@pytest.mark.parametrize('seed', [[float('nan')] * 6, [float('inf')] * 6, [0.] * 5])
def test_invalid_measured_seed_remains_rejected(seed):
    with pytest.raises(ValueError, match='finite position'):
        mapping().solve([0.] * 6, seed)


def test_closed_gripper_simulation_can_continue_with_measured_limit_overshoot():
    from robot_server_core.simulation import SimulationEnv
    mapper = mapping()
    env = SimulationEnv(cameras=False)
    try:
        env.activate('ik-gripper-regression')
        observation = env.observation()
        overshoot = False
        for _ in range(30):
            goal = mapper.solve([0.] * 6, observation.position, {'gripper': mapper.spec.lower[-1]})
            mapper.spec.validate(goal)
            observation, snapshot = env.step(goal)
            overshoot |= observation.position[-1] < mapper.spec.lower[-1]
            assert snapshot.status.mode == 'Running'
            assert all(trace.reference == goal.tolist() for trace in snapshot.trace)
        assert overshoot
    finally:
        env.close()


def test_frame_alignment_and_scale(tmp_path):
    text = (CONFIG / 'parol6_ik.toml').read_text().replace('translation_scale = 1.0', 'translation_scale = 2.0')
    text = text.replace('rotation_rpy = [0.0, 0.0, 0.0]', 'rotation_rpy = [0.0, 0.0, 1.5707963267948966]')
    config = tmp_path / 'aligned.toml'
    config.write_text(text)
    mapper = FKIKMapping(config)
    q = [.1, -.05, .1, .05, .02, 0.]
    leader = mapper.leader.pose(q)
    target = mapper.target_pose(q)
    rotation = (pin.rpy.rpyToMatrix(0., 0., np.pi / 2)
                @ mapper.follower_pose.rotation @ mapper.leader_pose.rotation.T)
    expected_translation = mapper.follower_pose.translation + 2 * rotation @ (leader.translation - mapper.leader_pose.translation)
    expected_rotation = rotation @ leader.rotation @ mapper.leader_pose.rotation.T @ rotation.T @ mapper.follower_pose.rotation
    assert_pose(target, pin.SE3(expected_rotation, expected_translation))
    assert_pose(mapper.target_pose(mapper.leader_reference), mapper.follower_pose)


def test_position_only_is_explicit(tmp_path):
    path = chain_urdf(tmp_path / 'chain.urdf', 3)
    model = UrdfModel(path, ['j0', 'j1', 'j2'], 'base', 'tool')
    q = [.2, .3, -.1]
    target = model.pose(q)
    target.rotation = pin.rpy.rpyToMatrix(1., 0., .4)
    solver = PoseIK(model, SolverSettings(orientation=False))
    goal = solver.solve(target, q)
    assert np.linalg.norm(model.pose(goal).translation - target.translation) < 1e-6


@pytest.mark.parametrize('before,after,message', [
    ('version = 1', 'version = 99', 'configuration version'),
    ('translation_scale = 1.0', 'translation_scale = -1.0', 'frame mapping'),
    ('direction = [1, 1, 1, 1, 1, 1]', 'direction = [2, 1, 1, 1, 1, 1]', 'encoder conversion'),
    ('open = 1.74533', 'open = 2.0', 'within its URDF bounds'),
    ('joint = "gripper"', 'joint = "wrist_roll"', 'outside the IK chain'),
])
def test_configuration_errors_fail_at_startup(tmp_path, before, after, message):
    config = tmp_path / 'invalid.toml'
    config.write_text((CONFIG / 'so101_ik.toml').read_text().replace(before, after))
    with pytest.raises(ValueError, match=message):
        FKIKMapping(config)
