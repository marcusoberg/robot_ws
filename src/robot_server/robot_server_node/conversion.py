"""ROS projections of public SDK observations, never a second state/control model."""

import json, time
from array import array
from robot_server.msg import Observation as ObservationMessage, Capture, CapturedImage
from sensor_msgs.msg import JointState, Image, CameraInfo, CompressedImage
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def header(target, time_ns, frame):
    target.stamp.sec, target.stamp.nanosec = divmod(int(time_ns), 1000000000)
    target.frame_id = frame


def observation(value):
    record = value.to_record()
    i = value.identity
    return ObservationMessage(
        runtime_instance=i.runtime_instance,
        timeline_generation=i.timeline_generation,
        observation_sequence=i.observation_sequence,
        execution_time_ns=int(record["time_ns"]),
        record_json=json.dumps(record),
    )


def capture(value, stream_sequence, host_clock):
    now = time.monotonic_ns()
    record = value.to_record()
    return Capture(
        runtime_instance=value.identity.runtime_instance,
        timeline_generation=value.epoch,
        observation_sequence=value.identity.observation_sequence,
        execution_time_ns=int(record["time_ns"]),
        delivery_sequence=stream_sequence,
        host_published_ns=now,
        host_clock=host_clock,
        observation_json=json.dumps(record),
        images=[CapturedImage(name=name, image=image(sample, int(record["time_ns"])),
                    calibration=calibration(sample, int(record["time_ns"])))
                for name, sample in value.images.items()],
    )


def joints(value, description):
    result = JointState()
    header(result.header, round(value.time * 1e9), description["base"])
    result.name = list(description["joints"])
    result.position = value.joints.position.tolist()
    result.velocity = value.joints.velocity.tolist()
    result.effort = (
        value.joints.effort.tolist() if value.joints.effort is not None else []
    )
    return result


def transform(parent, child, pose, time_ns):
    result = TransformStamped()
    header(result.header, time_ns, parent)
    result.child_frame_id = child
    (
        result.transform.translation.x,
        result.transform.translation.y,
        result.transform.translation.z,
    ) = map(float, pose.position)
    (
        result.transform.rotation.w,
        result.transform.rotation.x,
        result.transform.rotation.y,
        result.transform.rotation.z,
    ) = map(float, pose.quaternion_wxyz)
    return result


def frames(robot, value, *, exclude=()):
    return TFMessage(
        transforms=[
            transform(robot.description["base"], name, pose, round(value.time * 1e9))
            for name, pose in robot.frames(value).items()
            if name != robot.description["base"] and name not in exclude
        ]
    )


def image(sample, time_ns):
    result = Image()
    header(result.header, time_ns, sample.frame + "_optical")
    result.height, result.width = sample.pixels.shape[:2]
    result.encoding = "rgb8" if sample.pixels.ndim == 3 else "32FC1"
    result.is_bigendian = False
    result.step = result.width * (3 if sample.pixels.ndim == 3 else 4)
    result.data = array("B", sample.pixels.tobytes())
    return result


def calibration(sample, time_ns):
    result = CameraInfo()
    header(result.header, time_ns, sample.frame + "_optical")
    result.height, result.width = sample.pixels.shape[:2]
    result.k = sample.intrinsics.ravel().tolist()
    result.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    k = sample.intrinsics
    result.p = [
        float(k[0, 0]),
        0.0,
        float(k[0, 2]),
        0.0,
        0.0,
        float(k[1, 1]),
        float(k[1, 2]),
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    result.distortion_model = "plumb_bob"
    result.d = [0.0] * 5
    return result


def compressed_image(sample, time_ns):
    from PIL import Image as Pixels
    from io import BytesIO

    storage = BytesIO()
    Pixels.fromarray(sample.pixels).save(storage, format="PNG", compress_level=1)
    result = CompressedImage()
    header(result.header, time_ns, sample.frame + "_optical")
    result.format = "rgb8; png compressed rgb8"
    result.data = array("B", storage.getvalue())
    return result
