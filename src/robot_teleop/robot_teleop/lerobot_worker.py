"""LeRobot driver/dataset operations in the existing isolated model environment."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
import uuid
import numpy as np
import yaml
from robot_teleop import worker_pb2 as pb
from robot_teleop.wire import read, write
from robot_teleop.leader_fk import JOINTS

def leader(args):
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    device = SO101Leader(SO101LeaderConfig(port=args.port, id=args.leader_id,
        calibration_dir=Path(args.calibration_dir).expanduser() if args.calibration_dir else None, use_degrees=True))
    if args.calibrate:
        device.connect(calibrate=False)
        try:
            device.calibrate()
        finally:
            device.disconnect()
        return
    output = sys.stdout.buffer
    sys.stdout = sys.stderr
    try:
        if not device.calibration:
            raise ValueError("Leader calibration missing. Run the explicit calibrate command in a terminal first.")
        device.connect(calibrate=False)
        if not device.is_calibrated:
            raise ValueError("Leader calibration differs from device. Calibrate explicitly before teleoperation.")
        while True:
            value = device.get_action()
            angles = [value[j+".pos"] for j in JOINTS]
            grip = value["gripper.pos"]
            if not np.isfinite([*angles, grip]).all() or not -1 <= grip <= 101:
                raise ValueError("Invalid calibrated leader reading")
            write(output, pb.Reply(status="leader", leader_degrees=angles,
                gripper=grip, host_time_ns=time.monotonic_ns()))
            time.sleep(0.02)
    finally:
        if device.is_connected:
            device.disconnect()

def record(args):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    config = yaml.safe_load(Path(args.config).read_text())
    settings = config["recording"]
    binary_gripper = config.get("gripper", {"type": "binary"})["type"] == "binary"
    robot_type = settings.get("robot_type", "parol6_sim")
    root = Path(args.output).expanduser().resolve() / (
        robot_type+"-"+datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")+"-"+uuid.uuid4().hex[:6])
    output, source = sys.stdout.buffer, sys.stdin.buffer
    sys.stdout = sys.stderr
    dataset = None
    active = False
    previous = None
    count = 0
    manifest = None
    try:
        while True:
            request = read(source, pb.Request)
            try:
                op = request.operation
                if op == "start":
                    if active:
                        raise ValueError("Episode already active")
                    active, previous, count = True, None, 0
                    status = f"recording; dataset={root}"
                elif op == "frame":
                    if not active:
                        raise ValueError("No active episode")
                    f = request.frame
                    period = round(1e9 / settings["fps"])
                    if f.action_time_ns-f.time_ns != period:
                        raise ValueError("Frame does not span exactly one dataset interval")
                    if previous and (f.runtime, f.timeline, f.time_ns) != (previous[0], previous[1], previous[2]+period):
                        raise ValueError("Reset or missing simulation interval: discard this episode")
                    state, action = np.asarray(f.state, dtype=np.float32), np.asarray(f.action, dtype=np.float32)
                    if state.shape != (len(f.joints),) or action.shape != (len(f.joints)+int(binary_gripper),) or not np.isfinite(state).all() or not np.isfinite(action).all():
                        raise ValueError("Invalid robot state/action")
                    frame = {"observation.state": state, "action": action,
                             "task": settings["task"],
                             "collection.observation_time_ns": np.array([f.time_ns], dtype=np.int64),
                             "collection.action_time_ns": np.array([f.action_time_ns], dtype=np.int64)}
                    features = {
                        "observation.state": {"dtype":"float32","shape":(len(f.joints),),"names":list(f.joints)},
                        "action": {"dtype":"float32","shape":(len(f.joints)+int(binary_gripper),),"names":[*f.joints,*(["gripper_open"] if binary_gripper else [])]},
                        "collection.observation_time_ns": {"dtype":"int64","shape":(1,),"names":["ns"]},
                        "collection.action_time_ns": {"dtype":"int64","shape":(1,),"names":["ns"]}}
                    if len(f.images) != len(settings["cameras"]):
                        raise ValueError("Missing configured RGB cameras")
                    for image in f.images:
                        if image.name not in settings["cameras"] or len(image.rgb) != image.height*image.width*3:
                            raise ValueError("Invalid dataset image")
                        key = "observation.images."+image.name
                        frame[key] = np.frombuffer(image.rgb, dtype=np.uint8).reshape(image.height,image.width,3)
                        features[key] = {"dtype":"image","shape":(image.height,image.width,3),"names":["height","width","channels"]}
                    signature = json.dumps(features, sort_keys=True)
                    if dataset is None and manifest is not None:
                        dataset = LeRobotDataset.resume(repo_id="local/"+root.name, root=root, image_writer_threads=2)
                    if dataset is None:
                        dataset = LeRobotDataset.create(repo_id="local/"+root.name, fps=settings["fps"],
                            root=root, robot_type=robot_type, features=features, use_videos=False,
                            image_writer_threads=2, metadata_buffer_size=1)
                        (root/"teleoperation.json").write_text(json.dumps({
                            "config": config, "joint_units":"radians", "gripper_action":"0 closed, 1 open" if binary_gripper else "gripper joint radians",
                            "action_semantics":"native executed joint reference at t+1/fps; observation at t",
                            "gripper_feedback":"not part of observation.state" if binary_gripper else "included in observation.state",
                            "capture":json.loads(f.metadata_json)}, indent=2))
                        manifest = signature
                    if signature != manifest:
                        raise ValueError("Robot/camera schema changed during recording")
                    dataset.add_frame(frame)
                    previous = (f.runtime, f.timeline, f.time_ns)
                    count += 1
                    status = f"frame {count}"
                elif op in ("save", "discard"):
                    if not active:
                        raise ValueError("No active episode")
                    if op == "save":
                        if not count:
                            raise ValueError("Episode has no frames")
                        dataset.save_episode()
                        episode_index = dataset.num_episodes-1
                        dataset.finalize()
                        dataset = None
                        status = f"saved episode {episode_index}; frames={count}; dataset={root}"
                    else:
                        if dataset is not None:
                            dataset.clear_episode_buffer()
                        status = f"discarded; dataset={root}"
                    active, previous, count = False, None, 0
                elif op == "close":
                    if dataset is not None:
                        if active:
                            dataset.clear_episode_buffer()
                        dataset.finalize()
                        dataset = None
                    write(output, pb.Reply(status=f"closed; dataset={root}"))
                    return
                else:
                    raise ValueError("Unknown recording operation")
                write(output, pb.Reply(status=status))
            except Exception as error:
                write(output, pb.Reply(error=str(error)))
    finally:
        if dataset is not None:
            if active:
                dataset.clear_episode_buffer()
            dataset.finalize()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=("leader","record"))
    p.add_argument("--port")
    p.add_argument("--leader-id", default="so101-leader")
    p.add_argument("--calibration-dir")
    p.add_argument("--calibrate", action="store_true")
    p.add_argument("--config")
    p.add_argument("--output")
    a = p.parse_args()
    if a.mode == "leader":
        leader(a)
    else:
        record(a)

if __name__ == "__main__":
    main()
