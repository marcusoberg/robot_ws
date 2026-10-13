"""Inspect local checkpoint integrity; prediction is an explicit optional smoke run."""

import argparse
import hashlib
import json
import math
import os
import struct
from pathlib import Path


def inspect_checkpoint(directory):
    directory = Path(directory).resolve()
    weights = directory / "model.safetensors"
    with weights.open("rb") as stream:
        raw = stream.read(8)
        if len(raw) != 8:
            raise ValueError("Truncated Safetensors header")
        length = struct.unpack("<Q", raw)[0]
        if length > min(weights.stat().st_size - 8, 16 * 1024 * 1024):
            raise ValueError("Invalid Safetensors header length")
        header = json.loads(stream.read(length))

    byte_sizes = {
        "BOOL": 1,
        "U8": 1,
        "I8": 1,
        "I16": 2,
        "U16": 2,
        "I32": 4,
        "U32": 4,
        "I64": 8,
        "U64": 8,
        "F16": 2,
        "BF16": 2,
        "F32": 4,
        "F64": 8,
    }

    spans = []
    for name, value in header.items():
        if name == "__metadata__":
            continue
        a, b = value["data_offsets"]
        shape = value["shape"]
        if (
            value["dtype"] not in byte_sizes
            or any(type(n) is not int or n < 0 for n in shape)
            or b - a != math.prod(shape) * byte_sizes[value["dtype"]]
            or a < 0
        ):
            raise ValueError(f"Invalid tensor extent {name}")
        spans.append((a, b))

    cursor = 0
    for a, b in sorted(spans):
        if a != cursor:
            raise ValueError("Overlapping or missing tensor data")
        cursor = b
    if cursor + 8 + length != weights.stat().st_size:
        raise ValueError("Checkpoint file size differs from tensor extents")

    with weights.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()

    config = json.loads((directory / "config.json").read_text())

    required = ["policy_preprocessor.json", "policy_postprocessor.json"]
    for name in required:
        pipeline = json.loads((directory / name).read_text())
        for step in pipeline["steps"]:
            if step.get("state_file") and not (directory / step["state_file"]).is_file():
                raise ValueError("Missing processor state " + step["state_file"])

    state = directory.parent / "training_state"
    return {
        "checkpoint": str(directory),
        "model": config.get("type"),
        "sha256": digest,
        "bytes": weights.stat().st_size,
        "tensor_count": len(spans),
        "input_features": config.get("input_features"),
        "output_features": config.get("output_features"),
        "optimizer_step": json.loads((state / "training_step.json").read_text())["step"]
        if (state / "training_step.json").is_file() else None,
        "optimizer_state_present": (state / "optimizer_state.safetensors").is_file(),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", required=True, type=Path)
    p.add_argument("--features", type=Path)
    p.add_argument("--predict", action="store_true")
    p.add_argument("--device", choices=["cpu", "mps", "cuda"])
    p.add_argument("--offline", action="store_true")
    a = p.parse_args()
    result = inspect_checkpoint(a.checkpoint)
    if a.predict:
        if a.features is None or a.device is None:
            p.error("--predict requires --features and explicit --device")
        if a.offline:
            os.environ.update(
                HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_DATASETS_OFFLINE="1"
            )
        import numpy as np
        from model_api import Features, Identity, PolicyInput
        from .lerobot import LeRobotModel

        features = Features.model_validate_json(a.features.read_text())
        model = LeRobotModel.load(
            str(a.checkpoint),
            features=features,
            revision="local-smoke",
            device=a.device,
            model_type=result["model"],
        )
        value = PolicyInput(
            basis=Identity(
                runtime_instance="checkpoint-inspection",
                timeline_generation=0,
                observation_sequence=0,
            ),
            execution_time_ns=0,
            features={
                "observation.state": np.zeros(
                    len(features.state_joints), dtype=np.float32
                ),
                **{
                    name: np.zeros((shape[1], shape[2], shape[0]), dtype=np.uint8)
                    for name, shape in features.cameras.items()
                },
            },
            instruction="Pick up the cube and place it in the container.",
            episode="inspection",
            context="inspection",
        )
        prediction = model.predict(value)
        result["prediction"] = {
            "actions": len(prediction.actions),
            "finite": bool(np.isfinite(prediction.actions).all()),
            "computation_ns": prediction.computation_ns,
        }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
