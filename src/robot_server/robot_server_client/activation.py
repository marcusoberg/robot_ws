"""Nonblocking producer activation through the Robot Server's public ROS interface."""
from collections import OrderedDict
import time
import uuid
from robot_server.msg import CommandResult, MotionCommand
from robot_server.srv import Control


class Activation:
    def __init__(self, node, report):
        self.node, self.report = node, report
        self.client = node.create_client(Control, "/robot/control")
        self.results = OrderedDict()
        self.subscription = node.create_subscription(CommandResult, "/robot/result", self.result, 64)
        self.pending = None
        self.active = False
        self.active_epoch = None

    def epoch(self):
        identity = (self.node.state or {}).get("identity", {})
        return identity.get("runtime_instance"), identity.get("timeline_generation")

    def result(self, message):
        if message.request_id:
            key = (message.runtime_instance, message.timeline_generation, message.request_id)
            self.results[key] = message
            while len(self.results) > 64:
                self.results.popitem(last=False)

    def request(self, enabled):
        if not enabled:
            # A failed service lookup must never turn a local pause back into motion.
            self.active = False
            self.pending = None
            pause = self.node.command(MotionCommand.JOINT_POSITION, deadman=False)
            if pause is not None:
                self.node.commands.publish(pause)
        if not self.node.state or not self.client.service_is_ready():
            raise ValueError("Robot Server is not ready")
        if enabled and (self.active or self.pending):
            raise ValueError("Activation is already active or pending")
        if enabled and not any(s["source_instance"] == self.node.instance
                               for s in self.node.state.get("available_sources", [])):
            raise ValueError("Waiting for the server to register this producer's heartbeat")
        # Cancel local motion immediately, including when reversing an in-flight activation.
        self.active = False
        request = Control.Request(action=Control.Request.ACTIVATE if enabled else Control.Request.DEACTIVATE,
                                  source=self.node.source, source_instance=self.node.instance,
                                  request_key=str(uuid.uuid4()))
        self.pending = (self.client.call_async(request), self.epoch(), bool(enabled), time.monotonic()+10)
        self.report("Activating..." if enabled else "Pausing...")

    def tick(self):
        if self.active and (not self.node.selected or
                            self.active_epoch != (*self.epoch(), self.node.selected["session_generation"])):
            self.active = False
            self.report("Control lost or reset; activate again explicitly")
        if self.pending is None:
            return
        future, epoch, enabled, deadline = self.pending
        try:
            if epoch != self.epoch():
                raise ValueError("Robot reset or restarted; activate again explicitly")
            if time.monotonic() > deadline:
                self.pending = None
                if enabled:
                    self.request(False)
                self.report("Control request timed out; no new motion will be sent. Inspect /robot/result.")
                return
            if not future.done():
                return
            receipt = future.result()
            if not receipt.queued:
                raise ValueError(receipt.message)
            result = self.results.get((*epoch, receipt.request_id))
            if result is None or result.outcome in (CommandResult.QUEUED, CommandResult.ACCEPTED):
                return
            if result.outcome != CommandResult.COMPLETED:
                raise ValueError(result.reason)
            if enabled:
                selected = self.node.selected
                # Wait for the state publication carrying this exact native session.
                if not selected or selected["session_generation"] != result.session_generation:
                    return
                self.active_epoch = (*epoch, result.session_generation)
            self.active = enabled
            self.pending = None
            self.report("Active" if enabled else "Paused; robot holds position")
        except Exception as error:
            self.pending = None
            self.active = False
            self.report("Control request failed: " + str(error))
