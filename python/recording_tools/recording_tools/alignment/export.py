"""Deterministic derived datasets. Measured state at t, accepted reference at t+dt."""

from collections import defaultdict
from pathlib import Path
import hashlib
import json
import numpy as np
from mcap.reader import make_reader
from recording_tools.contracts import Event, ClockMapping
from recording_tools.recording.encoding import decode_capture, decode_capture_metadata


def camera_time_matches(record, cameras, mappings, budget_ns):
    """Resolve acquisition clocks during alignment, never from publication time."""
    state = record.observation["state"]
    destination = state["source_clock"]
    target = state["source_time_ns"]
    if target is None:
        return False, "robot acquisition time unavailable"
    metadata = record.observation["images"]
    for name in cameras.values():
        image = metadata.get(name)
        if image is None:
            return False, "camera metadata unavailable"
        stamp = image["stamp"]
        time_ns = stamp["source_time_ns"]
        if stamp["epoch"] != record.identity.timeline_generation or time_ns is None:
            return False, "camera acquisition time or timeline unavailable"
        clock = stamp["source_clock"]
        uncertainty = 0.0
        if clock != destination:
            choices = [
                m
                for m in mappings
                if m.source == clock
                and m.destination == destination
                and m.valid_from_ns <= time_ns
                and m.uncertainty_ns <= min(m.accuracy_budget_ns, budget_ns)
            ]
            if not choices:
                return False, "clock mapping unavailable or uncertainty exceeds budget"
            mapping = max(choices, key=lambda m: m.valid_from_ns)
            time_ns = mapping.convert(time_ns)
            uncertainty = mapping.uncertainty_ns
        if abs(time_ns - int(target)) + uncertainty > budget_ns:
            return False, "camera acquisition outside alignment budget"
    return True, "aligned"


def align(
    run: Path,
    *,
    period: float,
    cameras: dict[str, str],
    approved_only=True,
    accuracy_budget_ns=1000000,
    include_images=True,
):
    episodes = defaultdict(list)
    approvals = {}
    discontinuities = []
    mappings = []
    with (run / "recording.mcap").open("rb") as stream:
        for _, channel, msg in make_reader(stream).iter_messages(log_time_order=True):
            if channel.topic == "/experiment/events":
                event = Event.model_validate_json(msg.data)
                if event.kind == "clock.mapping":
                    mappings.append(ClockMapping.model_validate(event.details))
                if event.kind == "episode.finish" and event.episode:
                    approvals[event.episode.id] = event.episode
            elif channel.topic == "/robot/capture":
                if include_images:
                    record, images = decode_capture(channel.message_encoding, msg.data)
                else:
                    record = decode_capture_metadata(channel.message_encoding, msg.data)
                    images = None
                if record.episode:
                    episodes[record.episode.id].append(
                        (record, images, (channel.id, msg.sequence))
                    )
    aligned = {}
    dt = round(period * 1e9)
    for episode_id, samples in sorted(episodes.items()):
        episode = approvals.get(episode_id)
        if episode is None or (approved_only and not episode.approved):
            continue
        rows = []
        for (record, images, locator), (following, _, _) in zip(samples, samples[1:]):
            a, b = record.observation, following.observation
            delta = int(b["time_ns"]) - int(a["time_ns"])
            continuous = (
                record.identity.timeline_generation
                == following.identity.timeline_generation
                and a["generation"] == b["generation"]
                and a["owner"] == b["owner"]
                and abs(delta - dt) <= 1
                and a["state"] is not None
                and b["state"] is not None
            )
            camera_aligned, reason = (
                camera_time_matches(record, cameras, mappings, accuracy_budget_ns)
                if continuous
                else (False, "supervision/timing boundary")
            )
            if (
                not continuous
                or not camera_aligned
                or any(
                    camera not in record.observation["images"]
                    for camera in cameras.values()
                )
            ):
                if rows:
                    aligned[f"{episode_id}/{len(aligned)}"] = (episode, rows)
                    rows = []
                discontinuities.append(
                    {
                        "episode": episode_id,
                        "sequence": record.identity.observation_sequence,
                        "delta_ns": delta,
                        "reason": reason,
                    }
                )
                continue
            # Never substitute requested targets or next measured positions for accepted references.
            row = {
                "observation.state": np.asarray(a["state"]["position"], np.float32),
                "action": np.asarray(b["actuator_request"]["position"], np.float32),
                "task": episode.instruction,
                "basis": record.identity.model_dump(),
                "calibration": {
                    name: a["images"][name]["calibration"] for name in cameras.values()
                },
            }
            if include_images:
                row.update(
                    {feature: images[camera] for feature, camera in cameras.items()}
                )
            else:
                row["recording_locator"] = locator
                row["image_shapes"] = {
                    camera: record.observation["images"][camera]["shape"]
                    for camera in cameras.values()
                }
            rows.append(row)
        if rows:
            aligned[f"{episode_id}/{len(aligned)}"] = (episode, rows)
    return aligned, discontinuities


def export_lerobot(
    run: str | Path,
    destination: str | Path,
    *,
    period: float,
    cameras: dict,
    joints: list,
    seed=42,
    accuracy_budget_ns=1000000,
):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    run, destination = Path(run), Path(destination)
    manifest = json.loads((run / "manifest.json").read_text())
    if not manifest["complete"]:
        raise ValueError("Incomplete recording is excluded from baseline export")
    episodes, gaps = align(
        run,
        period=period,
        cameras=cameras,
        accuracy_budget_ns=accuracy_budget_ns,
        include_images=False,
    )
    if len(episodes) < 2:
        raise ValueError(
            "Need at least two approved complete segments for train/validation split"
        )
    # Split original episodes before any writer computes statistics; related recoveries stay together.
    groups = sorted({ep.parent or ep.id for ep, _ in episodes.values()})
    if len(groups) < 2:
        raise ValueError(
            "Need at least two approved original episodes/recovery groups for independent train/validation splits"
        )
    rng = np.random.default_rng(seed)
    rng.shuffle(groups)
    validation = set(groups[: max(1, len(groups) // 5)])
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(joints),),
            "names": joints,
        },
        "action": {"dtype": "float32", "shape": (len(joints),), "names": joints},
    }
    first = next(iter(episodes.values()))[1][0]
    for feature in cameras:
        features[feature] = {
            "dtype": "video",
            "shape": first["image_shapes"][cameras[feature]],
            "names": ["height", "width", "channels"],
        }
    counts = {}
    for split in ("train", "validation"):
        dataset = LeRobotDataset.create(
            repo_id=f"local/{destination.name}-{split}",
            fps=round(1 / period),
            root=destination / split,
            features=features,
            video_backend="pyav",
            robot_type=manifest["profile"]["robot_name"],
        )
        counts[split] = 0
        selected = {
            row["recording_locator"]: (key, row)
            for key, (ep, rows) in episodes.items()
            if ((ep.parent or ep.id) in validation) == (split == "validation")
            for row in rows
        }
        current = None
        # Sequential second pass keeps at most the writer's current episode images.
        # Pixel arrays from the entire dataset never remain resident together.
        with (run / "recording.mcap").open("rb") as stream:
            for _, channel, message in make_reader(stream).iter_messages(
                topics=["/robot/capture"], log_time_order=True
            ):
                entry = selected.get((channel.id, message.sequence))
                if entry is None:
                    continue
                key, row = entry
                if current is not None and current != key:
                    dataset.save_episode(parallel_encoding=False)
                    counts[split] += 1
                current = key
                _, images = decode_capture(channel.message_encoding, message.data)
                frame = {
                    k: v
                    for k, v in row.items()
                    if k
                    not in ("basis", "calibration", "recording_locator", "image_shapes")
                }
                frame.update(
                    {feature: images[camera] for feature, camera in cameras.items()}
                )
                dataset.add_frame(frame)
        if current is not None:
            dataset.save_episode(parallel_encoding=False)
            counts[split] += 1
        dataset.finalize()
    with (run / "recording.mcap").open("rb") as recording:
        source_hash = hashlib.file_digest(recording, "sha256").hexdigest()
    recipe = {
        "format": "robot.export.v1",
        "source_sha256": source_hash,
        "alignment": "measured state/images at t; accepted reference at t+action_period",
        "period": period,
        "cameras": cameras,
        "joints": joints,
        "seed": seed,
        "clock_accuracy_budget_ns": accuracy_budget_ns,
        "calibration_versions": {
            name: sorted(
                {
                    row["calibration"][name]
                    for _, rows in episodes.values()
                    for row in rows
                }
            )
            for name in cameras.values()
        },
        "validation_groups": sorted(validation),
        "episodes": counts,
        "gaps": gaps,
        "provenance": manifest,
        "encoding": {"backend": "pyav", "parallel_encoding": False},
        "normalization": "train writer statistics only; validation statistics never used for training",
    }
    (destination / "export.json").write_text(json.dumps(recipe, indent=2))
    return recipe


if __name__ == "__main__":
    from recording_tools.cli import main

    main()
