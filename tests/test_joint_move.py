import numpy as np
import pytest

from robot_description.spec import parol6
from robot_path.joint_move import JointMove


POSE = np.deg2rad([90., -144.683, 108.171, 5., 0., 90.])


@pytest.mark.parametrize('gripper', ['pneumatic', 'msg_ai_100'])
def test_timed_pose_reaches_exact_endpoint_and_preserves_gripper(gripper):
    profile = parol6(gripper, scene='robot_only')
    initial = np.r_[np.deg2rad([90., -90., 180., 0., 0., 180.]), profile.initial[-1]]
    move = JointMove(profile.spec, initial, POSE, 4.)
    samples = np.array([move.at(t) for t in np.linspace(0, 4, 401)])
    np.testing.assert_array_equal(samples[0], initial)
    np.testing.assert_array_equal(samples[-1, :6], POSE)
    np.testing.assert_array_equal(move.at(10), samples[-1])
    np.testing.assert_array_equal(move.at(-1), initial)
    assert move.duration == 4.
    assert not np.array_equal(move.at(3.99), move.target)
    assert np.all(samples[:, -1] == initial[-1])
    assert np.all(samples >= np.minimum(move.initial, move.target) - 1e-12)
    assert np.all(samples <= np.maximum(move.initial, move.target) + 1e-12)
    for sample in samples:
        profile.spec.validate(sample)
    # Starts and ends at rest; the interval is not stretched to a speed preset.
    dt = .0001
    assert np.max(np.abs((move.at(dt) - move.initial) / dt)) < 1e-6
    assert np.max(np.abs((move.target - move.at(4 - dt)) / dt)) < 1e-6
    np.testing.assert_array_equal(move.velocity_at(0), np.zeros(7))
    np.testing.assert_array_equal(move.velocity_at(4), np.zeros(7))
    for elapsed in (1., 2., 3.):
        difference = (move.at(elapsed + dt) - move.at(elapsed - dt)) / (2 * dt)
        np.testing.assert_allclose(move.velocity_at(elapsed), difference, atol=1e-8)
        assert move.velocity_at(elapsed)[-1] == 0


@pytest.mark.parametrize('duration', [0., -1., float('nan'), float('inf')])
def test_invalid_duration_is_rejected(duration):
    profile = parol6(scene='robot_only')
    with pytest.raises(ValueError, match='duration'):
        JointMove(profile.spec, profile.initial, POSE, duration)


@pytest.mark.parametrize('target', [[0.] * 5, [float('nan')] * 6, [10.] * 6])
def test_invalid_target_is_rejected(target):
    profile = parol6(scene='robot_only')
    with pytest.raises(ValueError):
        JointMove(profile.spec, profile.initial, target, 4.)
