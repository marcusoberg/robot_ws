"""Sample the requested Cartesian route. The server resolves joint motion."""

import math
import numpy as np

from robot_core.control.quintic import smooth_progress, smooth_rate
from robot_core.data.trajectory import CartesianPath
from .geometry import ShapeGeometry


class PathPlanner:
    def __init__(self, settings):
        self.settings = settings

    def plan(self, initial_xyz):
        settings = self.settings
        points = [initial_xyz]
        times = [0.]
        labels = ['start']
        for segment in ShapeGeometry(settings, initial_xyz).build():
            duration = max(settings.sample_period,
                           smooth_rate(0.5) * segment.length / settings.linear_rate,
                           smooth_rate(0.5) * segment.rotation_distance / settings.angular_rate) / settings.speed
            count = max(1, math.ceil(duration / settings.sample_period))
            if count > settings.maximum_segment_samples:
                raise ValueError(f'{segment.name}: too many samples for the configured geometry and timing')
            start = times[-1]
            for i in range(1, count + 1):
                fraction = i / count
                points.append(segment.position(smooth_progress(fraction)))
                times.append(start + fraction * duration)
                labels.append(segment.name)
        return CartesianPath(np.asarray(times), np.asarray(points), labels)
