"""Cartesian line, triangle and circle segments, expressed in base_link."""

from dataclasses import dataclass
import math
import numpy as np


# Peak derivative of 10u³ - 15u⁴ + 6u⁵ on [0, 1]. Used to enforce requested speed.
QUINTIC_PEAK_SPEED = 1.875


def smooth_progress(fraction):
    return fraction**3 * (10 - 15 * fraction + 6 * fraction**2)


def yaw_rotation(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])


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


class ShapeGeometry:
    def __init__(self, settings, initial_position):
        self.settings = settings
        self.position = np.array(initial_position, dtype=float)
        self.segments = []

    def transfer(self, name, destination):
        destination = np.asarray(destination, dtype=float)
        self.segments.append(TransferSegment(name, self.position, destination))
        self.position = destination

    def line(self, name, destination, heading):
        destination = np.asarray(destination, dtype=float)
        self.segments.append(LineSegment(name, self.position, destination, heading))
        self.position = destination

    def build(self):
        radius = self.settings.width / 2
        distance = self.settings.distance
        height = self.settings.height
        self.transfer("transfer_left", [0, distance, height])
        self.line("approach_line", [0, distance, height - radius], math.pi / 2)
        self.line("line_left", [0, distance, height + radius], math.pi / 2)
        self.transfer("transfer_front", [distance, 0, height])
        triangle_height = math.sqrt(3) * radius
        vertices = [
            np.array([distance, radius, height - triangle_height / 3]),
            np.array([distance, -radius, height - triangle_height / 3]),
            np.array([distance, 0, height + 2 * triangle_height / 3]),
        ]
        self.line("approach_triangle", vertices[0], 0)
        for vertex in vertices[1:] + vertices[:1]:
            self.line("triangle_front", vertex, 0)
        self.transfer("transfer_right", [0, -distance, height])
        self.line("approach_circle", [0, -distance, height + radius], -math.pi / 2)
        self.segments.append(
            CircleSegment("circle_right", np.array([0, -distance, height]), radius)
        )
        return self.segments
