from copy import deepcopy

import numpy as np
import pytest

import pinocchio as pin

from robot_core.config import robot_settings
from robot_description.spec import parol6, so101
from robot_core.kinematics import PoseIK, IKSettings, forward_kinematics, load_chain
from robot_teleop.config import tracking_profile
from robot_teleop.leader_pose import LeaderPose
from robot_core.kinematics.servo import CartesianServo


def pose(chain, positions):
    return forward_kinematics(chain, positions, chain.model.createData())


def assert_pose(actual, expected, position=1.01e-4, orientation=1.01e-3):
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
    model = load_chain(path, order, 'base', 'tool')
    natural_q = np.linspace(.1, .3, count)
    q = natural_q[::-1].copy()
    angles = np.cumsum(natural_q)
    lengths = np.array([.3] + [.2] * (count - 2) + [.1])
    expected_position = [np.dot(lengths, np.cos(angles)), np.dot(lengths, np.sin(angles)), 0.]
    expected = pin.SE3(pin.rpy.rpyToMatrix(0., 0., float(sum(natural_q))), np.array(expected_position))
    assert_pose(pose(model, q), expected)
    result = PoseIK(model, IKSettings.configured(max_iterations=250, position_tolerance=1e-9, orientation_tolerance=1e-9)).solve(expected, q + .02)
    assert np.all(result >= model.lower) and np.all(result <= model.upper)
    assert_pose(pose(model, result), expected)


def test_prismatic_joint_uses_metres(tmp_path):
    path = tmp_path / 'slider.urdf'
    path.write_text('''<robot name="slider"><link name="base"/><link name="tool"/>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>
        <origin xyz="0 0.1 0"/><axis xyz="1 0 0"/>
        <limit lower="-0.2" upper="0.2" effort="10" velocity="1"/>
      </joint></robot>''')
    model = load_chain(path, ['slide'], 'base', 'tool')
    assert model.units == ('m',)
    target = pin.SE3(np.eye(3), np.array([.12, .1, 0]))
    goal = PoseIK(model, IKSettings.configured(max_iterations=250, position_tolerance=1e-9, orientation_tolerance=1e-9)).solve(target, [0.])
    assert goal == pytest.approx([.12])


@pytest.mark.parametrize('name', ['so101', 'parol6', 'parol6_msg_ai_100'])
def test_engagement_anchors_at_measured_tcp(name):
    mapping = LeaderPose(tracking_profile(name))
    raw = dict.fromkeys(mapping.chain.joints, 2000)
    calibration = dict.fromkeys(raw, dict(range_min=100, range_max=3900))
    leader, grip = mapping.read(raw, calibration)
    tcp = pin.SE3(pin.rpy.rpyToMatrix(.2, -.4, .6), np.array([.13, .02, .21]))
    mapping.align(leader, tcp)
    assert_pose(mapping.target(leader), tcp)
    raw['shoulder_pan'] += 100
    moved, _ = mapping.read(raw, calibration)
    assert np.linalg.norm(mapping.target(moved).translation - tcp.translation) > .001
    if name == 'so101':
        raw['gripper'] = 100
        _, grip = mapping.read(raw, calibration)
        assert grip == pytest.approx([so101().spec.lower[-1]])
    else:
        assert grip == []


def test_servo_at_wrist_singularity_and_travel_stop_recovers():
    profile = parol6(scene='robot_only')
    chain = load_chain(profile.description, profile.spec.joints, 'base_link', 'tool_tcp')
    servo = CartesianServo(chain, robot_settings()['cartesian']['pneumatic'])
    q = np.r_[np.deg2rad(robot_settings()['motion']['pose_degrees']), profile.initial[-1]]
    start = servo.pose(q)
    dt = robot_settings()['motion']['status_period']
    outside = pin.SE3(start.rotation, start.translation + [2., 0., 0.])
    for target, count in [(outside, 150), (start, 1200)]:
        for _ in range(count):
            goal = servo.step(target, q, dt)
            assert np.all(np.abs(goal[:6] - q[:6]) <= servo.velocity_limits * dt + 1e-12)
            assert np.all(goal >= chain.lower) and np.all(goal <= chain.upper)
            assert goal[-1] == profile.initial[-1]
            q = goal
        if target is outside:
            assert np.linalg.norm(servo.pose(q).translation - start.translation) > .02
    assert_pose(servo.pose(q), start)


def test_relative_mapping_uses_rotated_axes_and_translation_scale():
    config = deepcopy(tracking_profile('parol6'))
    config['mapping'].update(translation_scale=2., rotation_rpy=[0., 0., np.pi / 2])
    mapper = LeaderPose(config)
    leader0 = pin.SE3.Identity()
    tcp = pin.SE3(pin.rpy.rpyToMatrix(.2, 0., 0.), np.array([.1, .2, .3]))
    mapper.align(leader0, tcp)
    leader = pin.SE3(pin.rpy.rpyToMatrix(0., .1, 0.), np.array([.01, 0., 0.]))
    axes = pin.rpy.rpyToMatrix(0., 0., np.pi / 2) @ tcp.rotation
    expected = pin.SE3(axes @ leader.rotation @ axes.T @ tcp.rotation,
                       tcp.translation + 2 * axes @ leader.translation)
    assert_pose(mapper.target(leader), expected)


def test_position_only_is_explicit(tmp_path):
    path = chain_urdf(tmp_path / 'chain.urdf', 3)
    model = load_chain(path, ['j0', 'j1', 'j2'], 'base', 'tool')
    q = [.2, .3, -.1]
    target = pose(model, q)
    target.rotation = pin.rpy.rpyToMatrix(1., 0., .4)
    solver = PoseIK(model)
    goal = solver.solve_position(target.translation, q)
    assert np.linalg.norm(pose(model, goal).translation - target.translation) < 1e-6
