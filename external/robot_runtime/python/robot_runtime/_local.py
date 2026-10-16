from functools import singledispatch
from .types import JointTarget, JointVelocity, Pose, Twist, Trajectory, ReferenceChunk, TimedJointTargets, Observation, RobotError

@singledispatch
def motion(value, n, base, epoch): raise TypeError(f"Unsupported motion type: {type(value).__name__}")
@motion.register(JointTarget)
def _(value, n, base, epoch):
    out = n.JointPosition(); out.values = value.values; return out
@motion.register(JointVelocity)
def _(value, n, base, epoch):
    out = n.JointVelocity(); out.values = value.values; return out
@motion.register(Pose)
def _(value, n, base, epoch):
    out = n.Pose(); out.frame = value.frame or base; out.position = value.position; out.quaternion_wxyz = value.quaternion_wxyz; out.auxiliary_positions = dict(value.auxiliary_positions); return out
@motion.register(Twist)
def _(value, n, base, epoch):
    out = n.Twist(); out.frame = value.frame or base; out.values = value.values; return out
@motion.register(Trajectory)
def _(value, n, base, epoch):
    out = n.Trajectory(); points = []
    for p in value.points:
        item = n.Waypoint(); item.position = p.position; item.velocity = p.velocity; item.offset = p.seconds; points.append(item)
    out.points = points; return out
@motion.register(ReferenceChunk)
def _(value, n, base, epoch):
    out = n.Schedule(); out.start_ns = round(value.start * 1e9); out.epoch = epoch; out.replaces = value.replaces.command_id if hasattr(value.replaces,"command_id") else value.replaces; points = []
    for p in value.points:
        item = n.ReferencePoint(); item.position = p.position; item.velocity = p.velocity; item.acceleration = p.acceleration; item.offset = p.seconds; points.append(item)
    out.points = points; return out

@motion.register(TimedJointTargets)
def _(value,n,base,epoch):
    out=n.TimedJointTargets();out.gripper_open=value.gripper_open;out.positions=value.positions;out.origin_ns=value.origin_ns;out.period_ns=value.period_ns
    out.start_ns=0 if value.start_ns is None else value.start_ns;out.lead_ns=value.lead_ns;out.join_seconds=value.join_seconds
    out.runtime_instance=value.basis.runtime_instance;out.epoch=value.basis.timeline_generation;out.observation_sequence=value.basis.observation_sequence
    out.tolerance=[value.fit_tolerance] if __import__("numpy").isscalar(value.fit_tolerance) else value.fit_tolerance
    out.replaces=value.predecessor.command_id if hasattr(value.predecessor,"command_id") else value.predecessor
    if value.joints is not None:
        order=n.NamedJointOrder();order.names=value.joints;out.order=order
    return out

class Local:
    def __init__(self,config,backend,mode,period):
        if backend=="mujoco":
            import importlib.util
            if importlib.util.find_spec("mujoco") is not None:
                import mujoco
        try:
            from robot_runtime_native import _native
        except ModuleNotFoundError as error:
            raise RobotError("Direct control needs the installed native SDK; run the root workspace setup") from error
        self.n=_native
        self.runtime=_native.Runtime(config._native(_native),backend,mode=="stepped",period)
        self.description=self.runtime.description
        self.operator_source=self.runtime.operator_source
        self.source=self.operator_source.source_id
    def observe(self):
        snapshot=self.runtime.snapshot();return Observation.from_data(snapshot.data,native=snapshot,origin=self)
    def operate(self,action): return self.operator_source.operate(getattr(self.n.Action,action))
    def wait(self,basis,timeout):self.runtime.wait(basis.epoch,basis.sequence,timeout)
    def close(self):self.runtime.close()
    def check_error(self,identifier):pass
    def capture(self,timeout):
        import time
        from .types import CameraError,ImageSample,SampleStamp,readonly
        deadline=time.monotonic()+timeout
        while True:
            captured=self.runtime.capture()
            if captured is not None:
                snapshot=captured["observation"]
                if int(snapshot.data["epoch"])==self.observe().epoch:break
            if time.monotonic()>=deadline:raise CameraError("No image capture available")
            time.sleep(0.002)
        import numpy as np
        images={}
        for name,item in captured["images"].items():
            shape=(item["height"],item["width"],3) if item["dtype"]=="uint8" else (item["height"],item["width"])
            dtype=np.dtype("u1") if item["dtype"]=="uint8" else np.dtype("<f4")
            images[name]=ImageSample(np.frombuffer(item["pixels"],dtype=dtype).reshape(shape),item["frame"],item["calibration"],SampleStamp.from_data(item),
                readonly(item["intrinsics"]).reshape(3,3),readonly(item["world_from_camera"]).reshape(4,4))
        data={**snapshot.data,"image_delivery":{"frame_sequence":captured["sequence"],"coalesced":0}}
        return Observation.from_data(data,native=snapshot,images=images,origin=self)
