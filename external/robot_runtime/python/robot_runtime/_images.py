"""Direct immutable capture publication; acquisition stays outside execution."""
from dataclasses import replace
from types import MappingProxyType
import numpy as np
from .types import ImageSample,CameraError,readonly
_DTYPES={"uint8":np.dtype("u1"),"float32":np.dtype("<f4")}
def publish_external(robot,samples,basis):
    if basis._native is None:raise CameraError("Use an observation from this local runtime")
    planes,storage,images=[],bytearray(),{}
    configured={s.name:s for s in robot.configuration.external_sensors}
    for name,sample in samples.items():
        desc=configured.get(name)
        if desc is None or desc.units:raise CameraError(f"{name!r} is not a configured image sensor")
        if sample.frame!=desc.frame or sample.calibration!=desc.calibration:raise CameraError("Image frame/calibration disagrees with configured sensor")
        pixels=np.asarray(sample.pixels)
        if pixels.dtype==np.uint8 and pixels.ndim==3 and pixels.shape[2]==3:dtype="uint8"
        elif pixels.dtype==np.float32 and pixels.ndim==2:dtype="float32"
        else:raise CameraError("Expected RGB uint8[H,W,3] or metric depth float32[H,W]")
        if sample.stamp.epoch!=basis.epoch:raise CameraError("Image and observation timelines disagree")
        raw=pixels.astype(_DTYPES[dtype],copy=False).tobytes();plane=robot._transport.n.ImagePlane()
        values=dict(name=name,dtype=dtype,shape=list(pixels.shape),offset=len(storage),bytes=len(raw),frame=desc.frame,calibration=desc.calibration,
            source_clock=sample.stamp.source_clock,source_time_ns=sample.stamp.source_time_ns,host_received_ns=sample.stamp.host_received_ns,
            sequence=sample.stamp.sequence,epoch=basis.epoch,intrinsics=sample.intrinsics.ravel().tolist(),world_from_camera=sample.world_from_camera.ravel().tolist())
        for key,value in values.items():setattr(plane,key,value)
        planes.append(plane);storage+=raw
        images[name]=ImageSample(np.frombuffer(raw,dtype=_DTYPES[dtype]).reshape(pixels.shape),desc.frame,desc.calibration,sample.stamp,readonly(sample.intrinsics),readonly(sample.world_from_camera))
    robot._transport.runtime.publish_capture(basis._native,planes,bytes(storage))
    return robot._transport.capture(0)
