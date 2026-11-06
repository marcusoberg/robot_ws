"""Cartesian path segment descriptions in base coordinates."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class LineSegment:
    name: str
    start: np.ndarray
    end: np.ndarray
    heading: float

    @property
    def length(self):
        return float(np.linalg.norm(self.end - self.start))

    @property
    def rotation_distance(self):
        return 0.0

    def position(self, progress):
        return self.start + progress * (self.end - self.start)

    def yaw(self, progress):
        return self.heading


@dataclass
class TransferSegment:
    name: str
    start: np.ndarray
    end: np.ndarray

    @property
    def start_heading(self):
        return math.atan2(self.start[1], self.start[0])

    @property
    def end_heading(self):
        return math.atan2(self.end[1], self.end[0])

    @property
    def rotation_distance(self):
        return abs(self.end_heading - self.start_heading)

    @property
    def length(self):
        start_radius = np.linalg.norm(self.start[:2])
        end_radius = np.linalg.norm(self.end[:2])
        return (
            max(start_radius, end_radius) * self.rotation_distance
            + abs(end_radius - start_radius)
            + abs(self.end[2] - self.start[2])
        )

    def position(self, progress):
        start_radius = np.linalg.norm(self.start[:2])
        end_radius = np.linalg.norm(self.end[:2])
        radius = start_radius + progress * (end_radius - start_radius)
        heading = self.yaw(progress)
        height = self.start[2] + progress * (self.end[2] - self.start[2])
        return np.array(
            [radius * math.cos(heading), radius * math.sin(heading), height]
        )

    def yaw(self, progress):
        return self.start_heading + progress * (self.end_heading - self.start_heading)


@dataclass
class CircleSegment:
    name: str
    centre: np.ndarray
    radius: float

    @property
    def length(self):
        return 2 * math.pi * self.radius

    @property
    def rotation_distance(self):
        return 0.0

    def position(self, progress):
        angle = 2 * math.pi * progress
        return self.centre + self.radius * np.array(
            [math.sin(angle), 0, math.cos(angle)]
        )

    def yaw(self, progress):
        return -math.pi / 2
