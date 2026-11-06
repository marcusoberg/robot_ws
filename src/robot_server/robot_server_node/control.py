"""Send one operator command; home waits for completion, not just acceptance."""

import argparse
import sys
import signal
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robot_core.config import robot_settings
from robot_server.msg import Control, Status


class ControlClient(Node):
    def __init__(self):
        super().__init__("robot_control", start_parameter_services=False)
        self.ros = robot_settings()["ros"]
        self.status = None
        self.instance = None
        self.publisher = self.create_publisher(Control, "/robot/control", robot_settings()["ros"]["command_depth"])
        self.create_subscription(Status, "/robot/status", self.receive_status, robot_settings()["ros"]["latest_depth"])

    def receive_status(self, status):
        if self.instance is None or status.instance == self.instance:
            self.status = status

    def send(self, operation):
        command_id = uuid.uuid4().hex
        self.publisher.publish(Control(
            command_id=command_id, instance=self.status.instance,
            episode=self.status.episode, session=self.status.session,
            operation=operation,
        ))
        return command_id

    def execute(self, operation):
        deadline = time.monotonic() + robot_settings()["motion"]["discovery_timeout"]
        while self.status is None or not self.publisher.get_subscription_count():
            if time.monotonic() >= deadline:
                raise RuntimeError("No robot server discovered")
            rclpy.spin_once(self, timeout_sec=self.ros["control_poll_period"])
        if operation == "home":
            self.instance = self.status.instance
        command_id = self.send(operation)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.ros["control_poll_period"])
            if self.status.command_id == command_id:
                if not self.status.command_accepted:
                    raise RuntimeError(self.status.command_result)
                print(self.status.command_result, flush=True)
                if operation == "home":
                    self.wait_for_home()
                return
        raise RuntimeError("No acknowledgement from the robot server")

    def wait_for_home(self):
        deadline = time.monotonic() + robot_settings()["hardware"]["home_timeout"]
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.ros["control_poll_period"])
            if self.status.mode == "Ready":
                print("Homing complete; motors remain enabled and holding", flush=True)
                return
            if self.status.mode in ("Fault", "Stopped", "Closed"):
                raise RuntimeError(f"Homing stopped: {self.status.reason or self.status.mode}")
        raise RuntimeError("Timed out waiting for homing completion")

    def cancel_home(self):
        if self.status is None:
            return
        command_id = self.send("disable")
        deadline = time.monotonic() + self.ros["cancel_home_timeout"]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.ros["ownership_poll_period"])
            if self.status.command_id == command_id and self.status.command_accepted:
                print("Server accepted disable request", file=sys.stderr)
                return
        print("Disable was requested but could not be confirmed", file=sys.stderr)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=(
        "record", "save", "discard", "reset", "pause", "enable", "disable", "home",
    ))
    args = parser.parse_args()
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, interrupt)
    node = ControlClient()
    failed = False
    try:
        node.execute(args.operation)
    except (KeyboardInterrupt, Exception) as error:
        failed = True
        print(str(error) or "Command interrupted", file=sys.stderr)
        if args.operation == "home" and rclpy.ok():
            try:
                node.cancel_home()
            except Exception as cancel_error:
                print(f"Could not confirm disable: {cancel_error}", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
