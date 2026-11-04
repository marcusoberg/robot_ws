#!/usr/bin/env python3
"""Runnable simulation placeholder with the server's command interface."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robot_server.msg import Control, JointGoal


class SimNode(Node):
    def __init__(self):
        super().__init__('robot_server', enable_rosout=False, start_parameter_services=False)
        self.create_subscription(Control, '/robot/control', self.on_control, 10)
        self.create_subscription(JointGoal, '/robot/joint_goal', self.on_joint_goal, 1)

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
