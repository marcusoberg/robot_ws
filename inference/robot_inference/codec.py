"""Typed Protobuf boundaries; no pickle, MessagePack or JSON network envelopes."""
import math
from dataclasses import asdict
import numpy as np
from google.protobuf.message import DecodeError
from . import protocol_pb2 as pb
from .contracts import Features, Identity, PolicyInput, Prediction

MAX_PACKET = 64 * 1024 * 1024
VERSION = 1

def envelope(kind, value, request_id=""):
    e = pb.Envelope(version=VERSION, request_id=request_id)
    getattr(e, kind).CopyFrom(value)
    data = e.SerializeToString()
    if len(data) > MAX_PACKET:
        raise ValueError("Inference message exceeds size limit")
    return data

def decode(data):
    if not isinstance(data, bytes) or len(data) > MAX_PACKET:
        raise ValueError("Expected a bounded binary Protobuf message")
    try:
        e = pb.Envelope.FromString(data)
    except DecodeError as error:
        raise ValueError("Invalid inference Protobuf") from error
    if e.version != VERSION or not e.WhichOneof("payload") or len(e.request_id) > 128:
        raise ValueError("Unsupported inference envelope")
    return e

def features_to_pb(f):
    value = pb.Features(**{k: v for k,v in asdict(f).items() if k != "cameras"})
    for name, (channels, height, width) in sorted(f.cameras.items()):
        value.cameras.add(name=name, channels=channels, height=height, width=width)
    return value

def features_from_pb(f):
    cameras = {v.name: (v.channels, v.height, v.width) for v in f.cameras}
    if len(cameras) != len(f.cameras):
        raise ValueError("Duplicate camera name")
    return Features(tuple(f.state_joints), cameras, f.action_period, f.instruction,
                    f.action_representation, tuple(f.action_joints), tuple(f.state_units), tuple(f.action_units), f.binary_gripper, f.action_time_offset_ns)

def identity(v):
    return Identity(v.runtime_instance, v.timeline_generation, v.observation_sequence)

def input_to_pb(v):
    result = pb.Input(basis=pb.Identity(**asdict(v.basis)), execution_time_ns=v.execution_time_ns,
                      instruction=v.instruction, episode=v.episode, context=v.context)
    for name, values in v.features.items():
        a = np.asarray(values)
        a = np.ascontiguousarray(a, dtype=np.uint8 if a.dtype == np.uint8 else "<f8")
        t = result.tensors.add(name=name, shape=a.shape, data=a.tobytes(),
                              dtype=pb.Tensor.UINT8 if a.dtype == np.uint8 else pb.Tensor.FLOAT64_LE)
        meta = v.capture_metadata.get("images", {}).get(name, {})
        stamp = meta.get("stamp", {})
        for key in ("sequence", "epoch", "source_clock", "source_time_ns", "host_received_ns"):
            if stamp.get(key) is not None:
                setattr(t.stamp, key, stamp[key])
        t.frame, t.calibration = meta.get("frame", ""), meta.get("calibration", "")
        t.intrinsics.extend(np.asarray(meta.get("intrinsics", [])).ravel())
        t.world_from_camera.extend(np.asarray(meta.get("world_from_camera", [])).ravel())
    return result

def input_from_pb(v):
    features, metadata = {}, {}
    for t in v.tensors:
        dtype = {pb.Tensor.UINT8: np.dtype("u1"), pb.Tensor.FLOAT64_LE: np.dtype("<f8")}.get(t.dtype)
        if dtype is None or t.name in features or not t.shape or len(t.shape) > 4:
            raise ValueError("Invalid tensor declaration")
        if any(n == 0 for n in t.shape) or math.prod(t.shape) * dtype.itemsize != len(t.data):
            raise ValueError("Tensor dimensions disagree with payload")
        features[t.name] = np.frombuffer(t.data, dtype=dtype).reshape(tuple(t.shape))
        metadata[t.name] = {"stamp": {"sequence": t.stamp.sequence, "epoch": t.stamp.epoch,
            "source_clock": t.stamp.source_clock,
            "source_time_ns": t.stamp.source_time_ns if t.stamp.HasField("source_time_ns") else None,
            "host_received_ns": t.stamp.host_received_ns},
            "frame": t.frame, "calibration": t.calibration, "intrinsics": list(t.intrinsics),
            "world_from_camera": list(t.world_from_camera)}
    return PolicyInput(identity(v.basis), v.execution_time_ns, features, v.instruction,
                       v.episode, v.context, {"images": metadata})

def prediction_to_pb(v):
    a = np.asarray(v.actions, dtype=float)
    return pb.Prediction(basis=pb.Identity(**asdict(v.basis)), execution_time_ns=v.execution_time_ns,
        model_revision=v.model_revision, action_representation=v.action_representation,
        action_period=v.action_period, steps=a.shape[0], dimensions=a.shape[1],
        actions=a.ravel().tolist(), computation_ns=v.computation_ns)

def prediction_from_pb(v):
    if not v.steps or not v.dimensions or v.steps * v.dimensions != len(v.actions):
        raise ValueError("Invalid prediction dimensions")
    return Prediction(identity(v.basis), v.execution_time_ns, v.model_revision,
        v.action_representation, v.action_period,
        np.asarray(v.actions).reshape(v.steps, v.dimensions).tolist(), v.computation_ns)
