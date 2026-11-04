"""Convert Cartesian geometry into a complete, checked joint trajectory.

This module performs geometry, IK and time parameterisation only. Backend state,
homing, source ownership and stopping belong to robot_server.
"""

from dataclasses import dataclass
import math
import numpy as np

from robot_description.spec import parol6
from robot_kinematics import IKSettings, PoseIK, UrdfModel
from .settings import HOMING_JOINT_SPEEDS
from .geometry import ShapeGeometry, QUINTIC_PEAK_SPEED, smooth_progress


@dataclass
class JointPath:
    times: np.ndarray
    joints: np.ndarray
    xyz: np.ndarray
    labels: list[str]
    position_error: float
    joint_speed: float
    joint_acceleration: float

    @property
    def duration(self):
        return float(self.times[-1])

    def at(self, elapsed):
        positions = [np.interp(elapsed, self.times, joint) for joint in self.joints.T]
        return np.asarray(positions)

    def velocity_at(self, elapsed):
        """Derivative of the same piecewise-linear path used by at()."""
        if elapsed <= self.times[0] or elapsed >= self.duration:
            return np.zeros(self.joints.shape[1])
        index = int(np.searchsorted(self.times, elapsed, side='right')) - 1
        return ((self.joints[index + 1] - self.joints[index])
                / (self.times[index + 1] - self.times[index]))

    def summary(self):
        return (
            f"{len(self.times)} samples, {self.duration:.1f} s; "
            f"sampled FK error {self.position_error * 1000:.3f} mm; "
            f"joint speed {self.joint_speed:.3f} rad/s; "
            f"sampled acceleration {self.joint_acceleration:.3f} rad/s²"
        )


class PathPlanner:
    def __init__(self, settings):
        settings.validate()
        self.settings = settings
        profile = parol6(settings.gripper, scene="robot_only")
        self.model = UrdfModel(
            profile.description,
            profile.spec.joints,
            "base_link",
            settings.tip,
        )
        if self.model.active.tolist() != list(range(6)):
            raise ValueError(
                "The tracing frame must use the six arm joints without gripper actuation"
            )
        self.solver = PoseIK(self.model, IKSettings(position_tolerance=1e-6))
        self.maximum_error = 0.0

    def solve_sample(self, target_position, seed):
        result = self.solver.solve_position(target_position, seed)
        error = float(
            np.linalg.norm(self.model.pose(result).translation - target_position)
        )
        if error > self.settings.position_tolerance:
            raise ValueError(
                f"IK cannot reach {target_position.tolist()}: residual {error:.6f} m"
            )
        self.maximum_error = max(self.maximum_error, error)
        return result

    def sample_segment(self, segment, seed, positions, points, names):
        duration = max(
            self.settings.sample_period,
            QUINTIC_PEAK_SPEED * segment.length / self.settings.sampling_linear_rate,
            QUINTIC_PEAK_SPEED * segment.rotation_distance / self.settings.sampling_angular_rate,
        )
        count = math.ceil(duration / self.settings.sample_period)
        if count > self.settings.maximum_segment_samples:
            raise ValueError(
                f"{segment.name}: too many IK samples for this geometry"
            )
        start_index = len(positions) - 1
        for index in range(1, count + 1):
            progress = smooth_progress(index / count)
            target = segment.position(progress)
            result = self.solve_sample(target, seed)
            midpoint = self.model.pose((seed + result) / 2).translation
            desired_midpoint = segment.position(smooth_progress((index - 0.5) / count))
            midpoint_error = float(np.linalg.norm(midpoint - desired_midpoint))
            if midpoint_error > self.settings.interpolation_tolerance:
                raise ValueError(
                    f"{segment.name}: joint interpolation error {midpoint_error:.6f} m"
                )
            self.maximum_error = max(self.maximum_error, midpoint_error)
            positions.append(result)
            points.append(target)
            names.append(segment.name)
            seed = result
        return start_index, len(positions) - 1

    def retime(self, joints, segments):
        delta = np.diff(joints[:, :6], axis=0)
        interval = self.settings.sample_period
        intervals = np.full(len(delta), interval)
        for first, last in segments:
            velocity = delta[first:last] / interval
            acceleration = (
                np.diff(np.vstack([np.zeros(6), velocity, np.zeros(6)]), axis=0)
                / interval
            )
            scale = max(
                np.max(np.abs(velocity) / np.asarray(HOMING_JOINT_SPEEDS)),
                math.sqrt(
                    np.max(np.abs(acceleration)) / self.settings.joint_acceleration
                ),
            )
            intervals[first:last] *= scale
        velocity = delta / intervals[:, None]
        boundary_intervals = (
            np.r_[intervals[0], intervals] + np.r_[intervals, intervals[-1]]
        ) / 2
        acceleration = (
            np.diff(np.vstack([np.zeros(6), velocity, np.zeros(6)]), axis=0)
            / boundary_intervals[:, None]
        )
        peak_velocity = float(np.max(np.abs(velocity)))
        peak_acceleration = float(np.max(np.abs(acceleration)))
        scale = max(
            1.0, math.sqrt(peak_acceleration / self.settings.joint_acceleration)
        )
        return (
            np.r_[0, np.cumsum(intervals * scale / self.settings.speed)],
            peak_velocity / scale * self.settings.speed,
            peak_acceleration / scale**2 * self.settings.speed**2,
        )

    def plan(self, initial_positions, progress=None):
        initial = self.model.validate(initial_positions)
        initial_pose = self.model.pose(initial)
        geometry = ShapeGeometry(self.settings, initial_pose.translation).build()
        joints = [initial]
        points = [initial_pose.translation.copy()]
        names = ["start"]
        boundaries = []
        for segment in geometry:
            boundary = self.sample_segment(
                segment, joints[-1], joints, points, names
            )
            boundaries.append(boundary)
            if progress is not None:
                progress(f"{segment.name}: reachable")
        joint_array = np.asarray(joints)
        times, speed, acceleration = self.retime(joint_array, boundaries)
        return JointPath(
            times,
            joint_array,
            np.asarray(points),
            names,
            self.maximum_error,
            speed,
            acceleration,
        )
