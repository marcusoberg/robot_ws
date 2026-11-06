"""Display the planned Cartesian route. These messages never command motion."""

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from robot_path.settings import path_settings


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
    style = path_settings()["markers"]
    markers = [line_marker("route", plan.xyz, style["route_color"], style["route_width"])]
    labels = np.asarray(plan.labels)
    for name, color in style["shapes"].items():
        indices = np.flatnonzero(labels == name)
        points = plan.xyz[np.r_[indices[0] - 1, indices]]
        markers.append(line_marker(name, points, color, style["shape_width"]))
    return MarkerArray(markers=markers)
