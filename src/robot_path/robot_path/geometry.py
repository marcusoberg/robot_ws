"""Cartesian line, triangle and circle segments, expressed in base_link."""

import math
import numpy as np

from robot_core.data.segments import CircleSegment, LineSegment, TransferSegment


def yaw_rotation(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])


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
