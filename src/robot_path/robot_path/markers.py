"""Display the planned Cartesian route. These messages never command motion."""

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


SHAPE_COLORS = {
    "line_left": (1.0, 0.35, 0.15),
    "triangle_front": (0.2, 1.0, 0.45),
    "circle_right": (0.2, 0.6, 1.0),
}


def line_marker(name, positions, color, width):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = name
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    marker.points = [
        Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        for point in positions
    ]
    return marker


def path_markers(plan):
    route_width = 0.001  # 1 mm transfer preview, overlaid by 3 mm colored shapes.
    shape_width = 0.003
    markers = [line_marker("route", plan.xyz, (0.5, 0.5, 0.5), route_width)]
    labels = np.asarray(plan.labels)
    for name, color in SHAPE_COLORS.items():
        indices = np.flatnonzero(labels == name)
        points = plan.xyz[np.r_[indices[0] - 1, indices]]
        markers.append(line_marker(name, points, color, shape_width))
    return MarkerArray(markers=markers)
