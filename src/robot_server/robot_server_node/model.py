#!/usr/bin/env python3
"""Publish URDF visual meshes as frame-locked markers for Foxglove."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import robot_description


def model_markers(description_file):
    description_file = Path(description_file)
    assets = Path(robot_description.__file__).parent / "assets"
    directory = description_file.parent.relative_to(assets).as_posix()
    root = ET.parse(description_file).getroot()
    markers = []
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            marker = Marker()
            marker.header.frame_id = link.attrib["name"]
            marker.ns = "robot_visuals"
            marker.id = len(markers)
            marker.frame_locked = True
            marker.action = Marker.ADD
            marker.type = Marker.MESH_RESOURCE
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError(f"Expected mesh geometry for {link.attrib['name']}")
            filename = mesh.attrib["filename"]
            marker.mesh_resource = filename if "://" in filename else (
                f"package://robot_description/{directory}/{filename}"
            )
            marker.scale.x, marker.scale.y, marker.scale.z = map(
                float, mesh.get("scale", "1 1 1").split()
            )
            origin = visual.find("origin")
            xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                float(value) for value in xyz.split()
            )
            roll, pitch, yaw = (float(angle) / 2 for angle in rpy.split())
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            marker.pose.orientation.x = sr * cp * cy - cr * sp * sy
            marker.pose.orientation.y = cr * sp * cy + sr * cp * sy
            marker.pose.orientation.z = cr * cp * sy - sr * sp * cy
            marker.pose.orientation.w = cr * cp * cy + sr * sp * sy
            color = visual.find("material/color")
            rgba = color.get("rgba") if color is not None else "0.75 0.75 0.75 1"
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                float(value) for value in rgba.split()
            )
            markers.append(marker)
    return MarkerArray(markers=markers)


class RobotModel(Node):
    def __init__(self):
        super().__init__("robot_visual_model")
        description = self.declare_parameter("description_file", "").value
        self.markers = model_markers(description)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/robot/model", qos)
        self.create_timer(1.0, self.publish_model)
        self.publish_model()

    def publish_model(self):
        self.publisher.publish(self.markers)


def main():
    rclpy.init()
    node = RobotModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
