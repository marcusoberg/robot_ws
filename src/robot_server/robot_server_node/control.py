import argparse
import time
import uuid

import rclpy
from rclpy.node import Node
from robot_server.msg import Control, Status


def main():
    parser = argparse.ArgumentParser(description='Publish an application control command over ROS')
    parser.add_argument('operation', choices=('record', 'save', 'discard', 'reset', 'pause', 'enable', 'disable'))
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('robot_control', start_parameter_services=False)
    statuses = []
    publisher = node.create_publisher(Control, '/robot/control', 10)
    node.create_subscription(Status, '/robot/status', lambda s: statuses.append(s), 1)
    command_id = uuid.uuid4().hex
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and (not statuses or not publisher.get_subscription_count()):
            rclpy.spin_once(node, timeout_sec=.1)
        if not statuses or not publisher.get_subscription_count():
            raise RuntimeError('No robot server discovered on /robot/status and /robot/control')
        publisher.publish(Control(command_id=command_id, instance=statuses[-1].instance, episode=statuses[-1].episode,
                                  session=statuses[-1].session, operation=args.operation))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if statuses[-1].command_id == command_id:
                if not statuses[-1].command_accepted:
                    raise RuntimeError(statuses[-1].command_result)
                print(statuses[-1].command_result)
                return
        raise RuntimeError('Command published, but no matching status received')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
