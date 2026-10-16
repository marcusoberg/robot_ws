"""Read processor coverage without loading policy weights or changing statistics."""

import json
import struct
from pathlib import Path


def normalization_report(checkpoint):
    root = Path(checkpoint)
    report = {}
    for filename, registry, feature in (
        ("policy_preprocessor.json", "normalizer_processor", "observation.state"),
        ("policy_postprocessor.json", "unnormalizer_processor", "action"),
    ):
        pipeline = json.loads((root / filename).read_text())
        steps = [step for step in pipeline["steps"] if step["registry_name"] == registry]
        if len(steps) != 1:
            raise ValueError(f"Expected one {registry} in {filename}")
        step = steps[0]
        kind = "STATE" if feature == "observation.state" else "ACTION"
        mode = step["config"]["norm_map"][kind]
        keys = []
        if step.get("state_file"):
            state_path = root / step["state_file"]
            if not state_path.resolve().is_relative_to(root.resolve()):
                raise ValueError("Processor state must stay inside its checkpoint directory")
            with state_path.open("rb") as stream:
                prefix = stream.read(8)
                if len(prefix) != 8:
                    raise ValueError("Truncated processor Safetensors header")
                length = struct.unpack("<Q", prefix)[0]
                if not 2 <= length <= min(state_path.stat().st_size - 8, 16 * 1024 * 1024):
                    raise ValueError("Invalid processor Safetensors header length")
                header = json.loads(stream.read(length))
                keys = [key for key in header if key != "__metadata__"]
        required = {
            "IDENTITY": (), "MEAN_STD": ("mean", "std"),
            "MIN_MAX": ("min", "max"), "QUANTILES": ("q01", "q99"),
            "QUANTILE10": ("q10", "q90"),
        }
        if mode not in required:
            raise ValueError(f"Unsupported normalization mode: {mode}")
        bound = feature in step["config"]["features"] and all(
            f"{feature}.{stat}" in keys for stat in required[mode]
        )
        report[feature] = {"mode": mode, "statistics_bound": bound, "available_keys": keys}
    return report
