from dataclasses import dataclass, field
import time
import uuid
import numpy as np
from robot_description.spec import so101

@dataclass(frozen=True)
class Observation:
    instance: str
    episode: int
    snapshot: int
    time_ns: int
    host_acquired_ns: int
    position: np.ndarray
    velocity: np.ndarray
    images: dict[str, np.ndarray]
    camera_metadata: dict = field(default_factory=dict)

    def features(self):
        return {"observation.state": self.position,
                **{f"observation.images.camera_{k}": v for k, v in self.images.items()}}

def immutable(values):
    a = np.array(values, copy=True)
    a.flags.writeable = False
    return a

def _render_worker(pipe, profile):
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(profile.scene))
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, height=profile.spec.image_shape[0], width=profile.spec.image_shape[1])
        pipe.send((True, None))
        while True:
            state = pipe.recv()
            if state is None:
                break
            mujoco.mj_setState(model, data, np.asarray(state), mujoco.mjtState.mjSTATE_INTEGRATION)
            mujoco.mj_forward(model, data)
            images, metadata = {}, {}
            for camera in profile.spec.cameras:
                renderer.update_scene(data, camera=camera)
                images[camera] = renderer.render().copy()
                camera_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,camera)
                h,w,_ = profile.spec.image_shape
                focal = h/(2*np.tan(np.deg2rad(model.cam_fovy[camera_id])/2))
                transform = np.eye(4)
                transform[:3,:3] = data.cam_xmat[camera_id].reshape(3,3) @ np.diag([1.,-1.,-1.])
                transform[:3,3] = data.cam_xpos[camera_id]
                quaternion = np.zeros(4)
                mujoco.mju_mat2Quat(quaternion,np.ascontiguousarray(transform[:3,:3]).reshape(9))
                metadata[camera] = dict(frame=f"{camera}_optical",intrinsics=[float(focal),0.,(w-1)/2,0.,float(focal),(h-1)/2,0.,0.,1.],
                                        world_from_camera=transform.tolist(),rotation_xyzw=quaternion[[1,2,3,0]].tolist())
            pipe.send((True, (images,metadata)))
        renderer.close()
    except BaseException as exc:
        try:
            pipe.send((False, str(exc)))
        except (BrokenPipeError, EOFError):
            pass
    finally:
        pipe.close()

class SnapshotRenderer:
    def __init__(self, profile):
        import multiprocessing as mp
        import threading
        self.lock = threading.Lock()
        context = mp.get_context("spawn")
        self.pipe, child = context.Pipe()
        self.process = context.Process(target=_render_worker, args=(child, profile), daemon=True)
        self.process.start()
        child.close()
        try:
            self._receive()
        except BaseException:
            self.close()
            raise

    def _receive(self):
        if not self.pipe.poll(30):
            raise TimeoutError("Snapshot renderer did not respond within 30 seconds")
        valid, value = self.pipe.recv()
        if not valid:
            raise RuntimeError(f"Snapshot renderer failed: {value}")
        return value

    def render(self, snapshot):
        with self.lock:
            self.pipe.send(snapshot.integration)
            images, metadata = self._receive()
            return {k: immutable(v) for k,v in images.items()}, metadata

    def close(self):
        with self.lock:
            if self.process.is_alive():
                try:
                    self.pipe.send(None)
                except (BrokenPipeError, EOFError):
                    pass
                self.process.join(timeout=5)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join()
            self.pipe.close()

class SimulationEnv:
    def __init__(self, profile=None, *, cameras=True):
        from robot_runtime_native._native import Simulation, StopProfile
        self.profile = profile or so101()
        self.instance = uuid.uuid4().hex
        stop = StopProfile()
        stop.velocity_threshold = self.profile.stop_velocity
        stop.settled_duration = self.profile.stop_settled_ns
        stop.timeout = self.profile.stop_timeout_ns
        self.native = Simulation(self.profile.native_spec(), str(self.profile.scene), self.profile.initial,
                                 self.profile.execution_period_ns, self.profile.spec.period_ns, stop, self.profile.physics_period_ns)
        self.renderer = SnapshotRenderer(self.profile) if cameras else None
        self.session = 0
        self.sequence = 0
        self.reset()

    def observation(self, snapshot=None):
        snap = snapshot or self.native.observe()
        acquired = snap.host_acquired_ns
        images, metadata = self.renderer.render(snap) if self.renderer else ({},{})
        return Observation(self.instance, snap.episode, snap.observation, snap.time, acquired,
                           immutable(snap.status.measured.position), immutable(snap.status.measured.velocity), images, metadata)

    def reset(self):
        if self.native.paced:
            self.native.stop_paced()
        snap = self.native.reset()
        self.native.enable()
        self.session = 0
        self.sequence = 0
        return self.observation(snap)

    def activate(self, source):
        self.session = self.native.activate(source)
        self.sequence = 0
        return self.session

    def step(self, action, *, session=None, sequence=None):
        goal = self.profile.spec.validate(action)
        next_sequence = self.sequence + 1 if sequence is None else sequence
        snapshot = self.native.step(goal.tolist(), self.session if session is None else session, next_sequence)
        self.sequence = next_sequence
        if snapshot.status.mode != "Running" or snapshot.status.delivered_sequence != self.sequence:
            raise RuntimeError(f"Action delivery failed: {snapshot.status.mode}: {snapshot.status.reason}")
        return self.observation(snapshot), snapshot

    def pause(self, reason="paused"):
        self.native.pause(reason)
        self.session = 0
        if not self.native.paced:
            while self.native.observe().status.mode == "Stopping":
                self.native.advance(1)
        else:
            while self.native.observe().status.mode == "Stopping":
                time.sleep(.005)
        return self.native.observe().status

    def close(self):
        self.native.stop_paced()
        self.native.disable()
        if self.renderer:
            self.renderer.close()
