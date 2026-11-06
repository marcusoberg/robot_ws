#!/usr/bin/env python3
"""Runnable simulation placeholder with the server's command interface."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robot_core.config import robot_settings
from robot_server.msg import Control, JointGoal, PoseGoal


class SimNode(Node):
    def __init__(self):
        super().__init__('robot_server', enable_rosout=False, start_parameter_services=False)
        self.create_subscription(Control, '/robot/control', self.on_control, robot_settings()["ros"]["command_depth"])
        self.create_subscription(JointGoal, '/robot/joint_goal', self.on_joint_goal, robot_settings()["ros"]["latest_depth"])

        self.create_subscription(PoseGoal, '/robot/pose_goal', self.on_pose_goal, robot_settings()["ros"]["latest_depth"])

    def on_pose_goal(self, message):
        pass

    def on_control(self, message):
        pass

    def on_joint_goal(self, message):
        pass


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
