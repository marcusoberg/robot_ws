#!/usr/bin/env python3
"""Attach a terminal to an existing robot server without reopening its backend."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from robot_server.msg import Status
from robot_description.spec import load_profile


class AttachedServer(Node):
    def __init__(self):
        super().__init__("robot_server_terminal", start_parameter_services=False)
        robot = self.declare_parameter("robot", "parol6").value
        gripper = self.declare_parameter("gripper", "pneumatic").value
        scene = self.declare_parameter("scene", "robot_only").value
        self.backend = self.declare_parameter("backend", "serial").value
        self.expected_instance = self.declare_parameter("instance", "").value
        profile = load_profile(robot, gripper=gripper, scene=scene)
        self.expected_spec = profile.spec.identity
        self.last_status = time.monotonic()
        self.last_state = None
        self.last_estop = None
        self.last_ownership = time.monotonic()
        self.create_subscription(Status, "/robot/status", self.status, 1)
        self.create_subscription(String, "/robot/ownership", self.ownership, 1)
        self.create_subscription(String, "/robot/serial_telemetry", self.telemetry, 1)
        self.create_timer(1.0, self.check_connection)

    def status(self, message):
        if message.action_spec_id != self.expected_spec:
            raise RuntimeError("Existing server uses a different robot configuration")
        if self.expected_instance and message.instance != self.expected_instance:
            raise RuntimeError("Server instance changed; run the task again to attach")
        self.expected_instance = message.instance
        self.last_status = time.monotonic()
        state = (message.mode, message.reason, message.command_id)
        if state != self.last_state:
            detail = message.reason
            if self.last_state is None or message.command_id != self.last_state[2]:
                detail = detail or message.command_result
            elif message.mode == "Ready" and self.last_state[0] == "Homing":
                detail = "Homing complete; motors holding"
            self.last_state = state
            print(f"Server {message.mode}: {detail}", flush=True)

    def telemetry(self, message):
        data = json.loads(message.data)
        state = (data["connected"], data["estop"], data["estop_verified"])
        if state != self.last_estop:
            self.last_estop = state
            print(
                f"Serial connected={state[0]}, E-stop pressed={state[1]}, "
                f"press/release verified={state[2]}",
                flush=True,
            )

    def ownership(self, message):
        feedback = json.loads(message.data)["feedback_kind"]
        actual_backend = "simulation" if feedback == "simulation" else "serial"
        if actual_backend != self.backend:
            raise RuntimeError(
                f"Existing server is {actual_backend}; requested {self.backend}"
            )
        self.last_ownership = time.monotonic()

    def check_connection(self):
        if time.monotonic() - min(self.last_status, self.last_ownership) > 5:
            raise RuntimeError("Existing server stopped publishing; run the task again")


def main():
    rclpy.init()
    node = None
    try:
        node = AttachedServer()
        print(
            "Attached to existing server. Ctrl-C detaches this terminal; "
            "use 'pixi run disable' to disable the robot.",
            flush=True,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f"Attachment ended: {error}", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
