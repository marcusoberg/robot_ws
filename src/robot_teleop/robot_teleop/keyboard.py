"""Keyboard producer: numbered joint, j/k jog, space clutch, d deadman, q quit."""

import sys
import argparse
import os
import select
import termios
import tty
import rclpy
from robot_server.msg import MotionCommand
from robot_server_client import Producer
from robot_server_client.activation import Activation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="keyboard")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not sys.stdin.isatty():
        parser.error("Keyboard control needs an interactive terminal.")
    rclpy.init()
    node = Producer(args.source)
    activation = Activation(node, lambda text: print(text, flush=True))
    saved = termios.tcgetattr(sys.stdin)
    index = 0
    clutch = False
    deadman = False
    was_selected = False
    try:
        node.wait_ready()
        q = node.positions()
        tty.setcbreak(sys.stdin.fileno())
        print(
            "1..9 joint; j/k +/-0.015; d toggles deadman; space clutch; q quits. Start with deadman OFF. d requests activation directly from the server.",
            flush=True,
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            activation.tick()
            selected = bool(node.selected)
            if selected and not was_selected:
                q = node.positions()
            was_selected = selected
            if select.select([sys.stdin], [], [], 0)[0]:
                key = os.read(sys.stdin.fileno(), 1).decode("ascii", errors="ignore")
                if key == "q":
                    break
                if key.isdigit() and 0 < int(key) <= len(q):
                    index = int(key) - 1
                if key == "d":
                    deadman = not (activation.active or activation.pending)
                    try:
                        activation.request(deadman)
                    except ValueError as error:
                        deadman = False
                        print(str(error), flush=True)
                    q = node.positions()
                if key == " ":
                    clutch = not clutch
                    q = node.positions()
                    print(f"clutch={clutch}", flush=True)
                if key in ("j", "k") and deadman and not clutch:
                    q[index] = min(
                        node.description["upper"][index],
                        max(
                            node.description["lower"][index],
                            q[index] + (0.015 if key == "k" else -0.015),
                        ),
                    )
            command = node.command(
                MotionCommand.JOINT_POSITION, deadman=deadman and activation.active, clutch=clutch
            )
            if command:
                command.values = q
                node.commands.publish(command)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.try_shutdown()
