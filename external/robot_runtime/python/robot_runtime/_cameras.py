"""Complete-state MuJoCo rendering mirror in a spawned acquisition process."""
from dataclasses import replace
from types import MappingProxyType
import hashlib
import math
import multiprocessing as mp
import threading
import time
import numpy as np
from .types import ImageSample, SampleStamp, CameraError, readonly


def apply_state(model, data, values):
    import mujoco
    mujoco.mj_setState(model, data, np.asarray(values, dtype=np.float64), mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)


def _render(pipe, scene, cameras):
    import mujoco
    renderers = []
    contexts = {}
    try:
        model = mujoco.MjModel.from_xml_path(scene)
        data = mujoco.MjData(model)
        for camera in cameras:
            identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera.name)
            if identifier < 0: raise CameraError(f"No scene camera named {camera.name!r}")
            if camera.width <= 0 or camera.height <= 0 or not math.isfinite(camera.frequency) or camera.frequency <= 0: raise CameraError("Camera extents and frequency must be positive")
            if model.cam_projection[identifier] == mujoco.mjtProjection.mjPROJ_ORTHOGRAPHIC: raise CameraError("RGB/depth capture requires a perspective camera")
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, camera.width)
            model.vis.global_.offheight = max(model.vis.global_.offheight, camera.height)
            extent = (camera.height, camera.width)
            renderer = contexts.get(extent)
            if renderer is None:
                renderer = mujoco.Renderer(model, height=camera.height, width=camera.width)
                contexts[extent] = renderer
            renderers.append((camera, identifier, renderer))
        pipe.send(("ready", None))
        while True:
            job = pipe.recv()
            if job is None: break
            values, stamp = job
            apply_state(model, data, values)
            planes, storage = [], bytearray()
            for camera, identifier, renderer in renderers:
                renderer.disable_depth_rendering(); renderer.update_scene(data, camera=camera.name)
                rgb = renderer.render().copy()
                outputs = [(camera.name + "/rgb", rgb, "uint8")]
                if camera.depth:
                    renderer.enable_depth_rendering(); renderer.update_scene(data, camera=camera.name)
                    outputs.append((camera.name + "/depth", renderer.render().astype("<f4"), "float32"))
                # Pixel coordinates refer to centres. MuJoCo's camera convention is
                # x right, y up, -z forward; world_from_camera preserves that frame.
                focal = camera.height / (2 * np.tan(np.deg2rad(model.cam_fovy[identifier]) / 2))
                fx, fy, cx, cy = focal, focal, (camera.width - 1) / 2, (camera.height - 1) / 2
                sensor_width, sensor_height = model.cam_sensorsize[identifier]
                if sensor_width > 0 and sensor_height > 0:
                    focal_x, focal_y, principal_x, principal_y = model.cam_intrinsic[identifier]
                    fx = focal_x * camera.width / sensor_width
                    fy = focal_y * camera.height / sensor_height
                    cx += principal_x * camera.width / sensor_width
                    cy -= principal_y * camera.height / sensor_height
                intrinsics = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                transform = np.eye(4); transform[:3, :3] = data.cam_xmat[identifier].reshape(3, 3); transform[:3, 3] = data.cam_xpos[identifier]
                calibration = camera.calibration or "sha256:" + hashlib.sha256(repr((camera.name, intrinsics)).encode()).hexdigest()
                for name, pixels, dtype in outputs:
                    raw = pixels.tobytes()
                    planes.append(dict(name=name, dtype=dtype, frame=camera.frame or camera.name, calibration=calibration,
                                       source_clock=stamp.source_clock, source_time_ns=stamp.source_time_ns, host_received_ns=stamp.host_received_ns, epoch=stamp.epoch, sequence=stamp.sequence,
                                       shape=list(pixels.shape), offset=len(storage), bytes=len(raw), intrinsics=intrinsics, world_from_camera=transform.ravel().tolist()))
                    storage += raw
            pipe.send(("frame", (planes, bytes(storage))))
    except (EOFError, BrokenPipeError): pass
    except Exception as error:
        try: pipe.send(("error", str(error)))
        except (OSError, EOFError): pass
    finally:
        for renderer in contexts.values(): renderer.close()
        pipe.close()


class Cameras:
    def __init__(self, robot, config):
        if robot.description["capabilities"]["backend"] != "mujoco": raise CameraError("Scene cameras require MuJoCo; publish external camera samples for hardware")
        self.robot = robot
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = None
        self._ready = threading.Event()
        self._frame_sequence = 0
        self._error = None
        context = mp.get_context("spawn")
        self._pipe, child = context.Pipe()
        self._process = context.Process(target=_render, args=(child, str(config.mujoco.model), config.cameras), name="robot-cameras", daemon=True)
        self._process.start(); child.close()
        try:
            if not self._pipe.poll(15): raise CameraError("Renderer did not initialize")
            kind, value = self._pipe.recv()
            if kind != "ready": raise CameraError(value)
        except BaseException: self.close(); raise
        self._worker = None
        if robot.mode == "realtime":
            self._worker = threading.Thread(target=self._run, args=(min(c.frequency for c in config.cameras),), name="robot-camera-handoff", daemon=True)
            self._worker.start()
    def _run(self, frequency):
        while not self._stop.is_set():
            began = time.monotonic()
            try: self.capture(self.robot.observe())
            except Exception as error:
                if not self._stop.is_set(): self._error = str(error)
                return
            self._stop.wait(max(0, 1 / frequency - (time.monotonic() - began)))
    def capture(self, observation, timeout=5):
        if self._error: raise CameraError(self._error)
        state = observation.sensors.get("mujoco/integration_state")
        if state is None: raise CameraError("Runtime does not provide complete MuJoCo integration state")
        # One in-flight render; producer and explicit capture coalesce rather than
        # accumulating old scenes. No graph or backend lock is held while rendering.
        if not self._lock.acquire(timeout=timeout): raise CameraError("Camera worker is busy")
        try:
            self._pipe.send((state.values, state.stamp))
            if not self._pipe.poll(timeout):
                self._error = "Camera capture timed out"; raise CameraError(self._error)
            kind, value = self._pipe.recv()
            if kind != "frame": raise CameraError(value)
            planes, compressed = value
            captured = self.publish(observation, planes, compressed)
            self._latest = captured
            self._ready.set()
            return captured
        finally: self._lock.release()
    def publish(self, basis, planes, compressed):
        from ._images import _DTYPES
        storage = compressed
        images, native_planes = {}, []
        for plane in planes:
            pixels = np.frombuffer(storage, dtype=_DTYPES[plane["dtype"]], count=plane["bytes"] // _DTYPES[plane["dtype"]].itemsize, offset=plane["offset"]).reshape(plane["shape"])
            images[plane["name"]] = ImageSample(pixels, plane["frame"], plane["calibration"], SampleStamp.from_data(plane),
                                              readonly(plane["intrinsics"]).reshape(3, 3), readonly(plane["world_from_camera"]).reshape(4, 4))
            native = self.robot._transport.n.ImagePlane()
            for key, value in plane.items(): setattr(native, key, value)
            native_planes.append(native)
        self.robot._transport.runtime.publish_capture(basis._native, native_planes, compressed)
        from .types import freeze
        self._frame_sequence += 1
        data = {**basis._data, "image_delivery": {"frame_sequence": self._frame_sequence, "coalesced": 0}}
        return replace(basis, images=MappingProxyType(images), _data=freeze(data))
    def latest(self, timeout=5):
        """Immutable latest capture from the bounded live acquisition worker."""
        if not self._ready.wait(timeout):
            raise CameraError(self._error or "No camera capture available")
        if self._error: raise CameraError(self._error)
        return self._latest
    def clear(self):
        self._latest = None
        self._ready.clear()
    def close(self):
        self._stop.set()
        worker = getattr(self, "_worker", None)
        if worker: worker.join(timeout=6)
        if hasattr(self, "_pipe"):
            try: self._pipe.send(None)
            except (OSError, EOFError): pass
        if hasattr(self, "_process"):
            self._process.join(timeout=3)
            if self._process.is_alive(): self._process.terminate(); self._process.join(timeout=3)
        if hasattr(self, "_pipe"): self._pipe.close()
