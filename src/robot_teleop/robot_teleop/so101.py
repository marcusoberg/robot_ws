"""SO101 handle -> Cartesian ROS targets; optional native LeRobot episode writer."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from robot_server.msg import Capture, MotionCommand
from robot_server.srv import Step, CaptureOnce
from robot_server_client import Producer
from . import worker_pb2 as pb
from .wire import read, write
from .leader_fk import LeaderFK

class SO101Teleop(Producer):
    def __init__(self, args):
        self.leader_sample = None
        self.engaged = False
        self.fault = None
        self.engaged_epoch = None
        super().__init__("so101")
        self.config = yaml.safe_load(Path(args.config).read_text())
        self.fk = LeaderFK(args.urdf, self.config["mapping"])
        self.fps = self.config["recording"]["fps"]
        if type(self.fps) is not int or not 1 <= self.fps <= 50:
            raise ValueError("Recording fps must be an integer between 1 and 50")
        if 1_000_000_000 % self.fps:
            raise ValueError("Recording FPS must divide one second in nanoseconds exactly")
        self.period_ns = 1_000_000_000 // self.fps
        self.gripper_threshold = self.config.get("gripper_open_threshold", 50)
        if not 0 <= self.gripper_threshold <= 100:
            raise ValueError("Gripper threshold must be in [0,100]")
        self.recording = False
        self.pending_operation = None
        self.job = None
        self.job_operation = None
        self.step_future = None
        self.repeat_future = None
        self.wait_capture = None
        self.previous = None
        self.captures = {}
        self.last_step = 0.0
        self.last_pause = False
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lerobot_writer")
        self.status = self.create_publisher(String, "/teleop/status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.targets = self.create_publisher(PoseStamped, "/teleop/target", 1)
        self.create_subscription(Capture, "/robot/capture", self.receive_capture, 64)
        self.steps = self.create_client(Step, "/robot/step")
        self.capture_once = self.create_client(CaptureOnce, "/robot/capture_once")
        self.create_service(SetBool, "/teleop/engage", self.engage)
        for op in ("start", "save", "discard"):
            self.create_service(Trigger, "/teleop/record/"+op,
                lambda request, response, op=op: self.record_control(op, response))
        env = dict(os.environ)
        for key in ("PYTHONPATH","PYTHONHOME","LD_LIBRARY_PATH","DYLD_LIBRARY_PATH",
                    "DYLD_FALLBACK_LIBRARY_PATH","AMENT_PREFIX_PATH","CMAKE_PREFIX_PATH"):
            env.pop(key, None)
        env["PYTHONPATH"] = str(Path(__file__).resolve().parent.parent)
        common = [args.python, "-m", "robot_teleop.lerobot_worker"]
        leader_args = ["leader", "--port", args.port, "--leader-id", args.leader_id]
        if args.calibration_dir:
            leader_args += ["--calibration-dir", args.calibration_dir]
        self.leader_process = subprocess.Popen(common+leader_args, stdout=subprocess.PIPE, env=env)
        self.writer_process = subprocess.Popen(common+["record","--config",args.config,"--output",args.output],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        self.reader = threading.Thread(target=self.read_leader, daemon=True)
        self.reader.start()
        self.create_timer(0.01, self.tick)
        self.report("disengaged; select so101 and enable explicitly; /teleop/target previews the mapped pose")

    def report(self, text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def epoch(self):
        i = (self.state or {}).get("identity", {})
        return (i.get("runtime_instance"), i.get("timeline_generation"), (self.selected or {}).get("session_generation"))

    def healthy(self):
        return self.leader_sample is not None and 0 <= time.monotonic_ns()-self.leader_sample.host_time_ns < 300_000_000

    def heartbeat(self):
        # A live ROS process cannot renew authority on behalf of a stalled USB reader.
        if self.healthy():
            super().heartbeat()

    def read_leader(self):
        try:
            while True:
                sample = read(self.leader_process.stdout, pb.Reply)
                if sample.error or len(sample.leader_degrees) != 5:
                    raise ValueError(sample.error or "Invalid leader sample")
                self.leader_sample = sample
        except Exception as error:
            self.leader_sample = None
            self.get_logger().error("Leader disconnected: "+str(error))

    def engage(self, request, response):
        if request.data and (not self.healthy() or not self.selected or self.fault):
            response.success, response.message = False, "Fresh leader, explicit selection and no recording fault are required"
        else:
            self.engaged = request.data
            self.engaged_epoch = self.epoch() if request.data else None
            response.success, response.message = True, "engaged" if self.engaged else "disengaged"
            self.report(response.message)
        return response

    def record_control(self, op, response):
        if self.pending_operation or (self.job and self.job_operation != "frame"):
            response.success, response.message = False, "Recording operation already pending"
        elif op == "start" and (self.recording or not self.engaged or not self.selected or not self.healthy() or self.fault):
            response.success, response.message = False, "Engage the selected, healthy source first"
        elif op != "start" and not self.recording:
            response.success, response.message = False, "No episode is active"
        elif op == "save" and self.fault:
            response.success, response.message = False, "Episode invalid: discard it"
        else:
            self.pending_operation = op
            response.success, response.message = True, "queued; await /teleop/status acknowledgement"
        return response

    def receive_capture(self, msg):
        self.captures[(msg.runtime_instance,msg.timeline_generation,msg.execution_time_ns)] = msg
        while len(self.captures) > 32:
            self.captures.pop(next(iter(self.captures)))

    def exchange(self, request):
        write(self.writer_process.stdin, request)
        reply = read(self.writer_process.stdout, pb.Reply)
        if reply.error:
            raise RuntimeError(reply.error)
        return reply.status

    def submit(self, op, frame=None):
        request = pb.Request(operation=op)
        if frame is not None:
            request.frame.CopyFrom(frame)
        self.job_operation = op
        self.job = self.pool.submit(self.exchange, request)

    def frame(self, before, after):
        a, b = json.loads(before.observation_json), json.loads(after.observation_json)
        if after.execution_time_ns-before.execution_time_ns != self.period_ns:
            raise ValueError("Missing simulation interval")
        for message, record in ((before,a),(after,b)):
            identity = record["identity"]
            if (identity["runtime_instance"],identity["timeline_generation"],identity["observation_sequence"],record["time_ns"]) != (
                message.runtime_instance,message.timeline_generation,message.observation_sequence,message.execution_time_ns):
                raise ValueError("Capture metadata disagrees with ROS header")
        if (before.runtime_instance,before.timeline_generation) != (after.runtime_instance,after.timeline_generation):
            raise ValueError("Simulation reset during episode")
        if b["actuator_request"]["gripper_open"] is None:
            raise ValueError("No native gripper command has been observed")
        value = pb.Frame(state=a["state"]["position"],
            action=[*b["actuator_request"]["position"], float(b["actuator_request"]["gripper_open"])],
            joints=self.description["joints"], runtime=before.runtime_instance, timeline=before.timeline_generation,
            observation=before.observation_sequence, time_ns=before.execution_time_ns,
            action_time_ns=after.execution_time_ns, metadata_json=before.observation_json)
        planes = {p.name:p.image for p in before.images}
        for feature, source in self.config["recording"]["cameras"].items():
            img = planes[source]
            stamp = a["images"][source]["stamp"]
            if stamp["source_time_ns"] != before.execution_time_ns:
                raise ValueError("Camera and joint observation are not synchronized")
            if img.encoding != "rgb8" or img.is_bigendian or img.step != img.width*3 or len(img.data) != img.height*img.step:
                raise ValueError("Recording requires packed RGB8 camera planes")
            value.images.add(name=feature,height=img.height,width=img.width,rgb=bytes(img.data))
        return value

    def motion(self):
        ready = self.healthy() and bool(self.selected) and self.engaged and not self.fault
        if self.healthy() and self.description:
            mapped = self.fk.pose(self.leader_sample.leader_degrees)
            preview = PoseStamped()
            preview.header.frame_id = self.description["base"]
            preview.header.stamp.sec, preview.header.stamp.nanosec = divmod(self.state["time_ns"], 1_000_000_000)
            preview.pose.position.x, preview.pose.position.y, preview.pose.position.z = map(float,mapped[:3,3])
            q = Rotation.from_matrix(mapped[:3,:3]).as_quat()
            preview.pose.orientation.x,preview.pose.orientation.y,preview.pose.orientation.z,preview.pose.orientation.w = map(float,q)
            self.targets.publish(preview)
        if ready:
            command = self.command(MotionCommand.BASE_POSE)
            command.frame_id, command.pose = self.description["base"], preview.pose
            command.has_gripper_command = True
            command.gripper_open = self.leader_sample.gripper >= self.gripper_threshold
            self.commands.publish(command)
            self.last_pause = False
        elif self.selected and not self.last_pause:
            command = self.command(MotionCommand.BASE_POSE, deadman=False)
            self.commands.publish(command)
            self.last_pause = True

    def tick(self):
        try:
            if self.job:
                if not self.job.done():
                    return
                job, op = self.job, self.job_operation
                self.job = None
                text = job.result()
                if op == "start":
                    self.recording, self.previous = True, None
                elif op in ("save","discard"):
                    self.recording, self.previous, self.fault = False, None, None
                if op != "frame":
                    self.report(text)
            if not self.state or not self.description:
                return
            if self.engaged and self.engaged_epoch != self.epoch():
                self.engaged = False
                self.report("Control session changed; engage explicitly again")
            server = self.description.get("server", {})
            if server.get("mode") != "stepped" or server.get("backend") != "mujoco":
                raise ValueError("This collection launch requires stepped MuJoCo; it never drives a physical follower")
            if self.step_future:
                if not self.step_future.done():
                    if time.monotonic()-self.last_step > 10:
                        raise TimeoutError("Simulation step timed out; do not issue another step")
                    return
                reply = self.step_future.result()
                self.step_future = None
                if not reply.success:
                    raise ValueError(reply.error)
                self.wait_capture = (reply.runtime_instance,reply.timeline_generation,reply.execution_time_ns)
                self.capture_deadline = time.monotonic()
            if self.wait_capture:
                captured = self.captures.get(self.wait_capture)
                if captured is None:
                    if time.monotonic()-self.capture_deadline > 0.5 and self.repeat_future is None:
                        self.repeat_future = self.capture_once.call_async(CaptureOnce.Request(request_key=str(uuid.uuid4())))
                    if time.monotonic()-self.capture_deadline > 5:
                        raise TimeoutError("Final step capture missing; episode must be discarded")
                    return
                self.wait_capture, self.repeat_future = None, None
                if self.recording and not self.fault and self.pending_operation not in ("save", "discard"):
                    if not self.engaged or not self.selected or not self.healthy():
                        raise ValueError("Leader/authority lost during episode; discard episode")
                    if self.previous is not None:
                        self.submit("frame", self.frame(self.previous,captured))
                    self.previous = captured
                    if self.job:
                        return
            if self.pending_operation:
                op, self.pending_operation = self.pending_operation, None
                if op in ("save","discard"):
                    self.engaged = False
                    self.motion()
                self.submit(op)
                return
            if self.fault:
                self.motion()
                return
            if time.monotonic()-self.last_step < 1/self.fps or not self.steps.service_is_ready():
                return
            self.motion()
            # Step.n counts SDK action periods, not internal controller ticks.
            dt = round(float(server["action_period"])*1e9)
            if dt <= 0 or self.period_ns % dt or not 1 <= self.period_ns//dt <= 1000:
                raise ValueError("Dataset period must be a whole number of server action periods")
            self.last_step = time.monotonic()
            self.step_future = self.steps.call_async(Step.Request(n=self.period_ns//dt,request_key=str(uuid.uuid4())))
        except Exception as error:
            if not self.fault:
                self.fault = str(error)
                self.engaged = False
                self.report("ERROR: "+self.fault+"; discard the episode before continuing")
                self.motion()

    def close(self):
        self.engaged = False
        try:
            self.motion()
        except Exception:
            pass
        self.leader_process.terminate()
        try:
            self.leader_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.leader_process.kill()
            self.leader_process.wait()
        try:
            self.pool.submit(self.exchange, pb.Request(operation="close")).result(timeout=30)
        except Exception:
            self.writer_process.terminate()
        try:
            self.writer_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.writer_process.kill()
            self.writer_process.wait()
        self.pool.shutdown(wait=False, cancel_futures=True)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("config","urdf","python","output","port"):
        p.add_argument("--"+name, required=True)
    p.add_argument("--leader-id", default="so101-leader")
    p.add_argument("--calibration-dir")
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = None
    try:
        node = SO101Teleop(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
