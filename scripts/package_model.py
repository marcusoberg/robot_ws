"""Extract a portable inference bundle without importing Torch or loading tensors.

Keep the complete checkpoint separately when optimizer resume is required.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python/model_server"))
from model_server.inspect import inspect_checkpoint


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def local_file(root, relative):
    path = root / relative
    if Path(relative).is_absolute() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"Resource escapes its directory: {relative}")
    if not path.is_file():
        raise ValueError(f"Missing resource: {path}")
    return path


def copy_file(source, target):
    # Independent copy-on-write files where supported; never mutable hard links.
    command = (["cp", "-c"] if sys.platform == "darwin" else ["cp", "--reflink=auto"])
    try:
        subprocess.run([*command, str(source), str(target)], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        shutil.copy2(source, target)
    if digest(Path(source)) != digest(Path(target)):
        raise ValueError(f"Copy hash mismatch: {source}")
    return str(target)


def package(checkpoint, features, output, *, hf_cache=None, expected_sha256=None, label=None):
    checkpoint, output = Path(checkpoint).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError(f"Output already exists; choose a new directory: {output}")
    inspection = inspect_checkpoint(checkpoint)
    if expected_sha256 and inspection["sha256"] != expected_sha256:
        raise ValueError("Checkpoint hash differs from the selected qualification evidence")
    config = json.loads((checkpoint / "config.json").read_text())
    contract = json.loads(Path(features).read_text())
    expected = {"observation.state": [len(contract["state_joints"])], **contract["cameras"]}
    actual = {k: v["shape"] for k, v in config["input_features"].items()}
    if actual != expected or config["output_features"]["action"]["shape"] != [len(contract["state_joints"])]:
        raise ValueError("Feature contract differs from checkpoint")
    names = {"model.safetensors", "config.json", "policy_preprocessor.json", "policy_postprocessor.json"}
    repositories = set()
    if config.get("type") == "smolvla":
        repositories.add(config["vlm_model_name"])
    for pipeline in ("policy_preprocessor.json", "policy_postprocessor.json"):
        for step in json.loads((checkpoint / pipeline).read_text())["steps"]:
            if step.get("state_file"):
                names.add(step["state_file"])
            tokenizer = step.get("config", {}).get("tokenizer_name")
            if tokenizer:
                repositories.add(tokenizer)
    for name in names:
        local_file(checkpoint, name)
    caches = []
    for repo in sorted(repositories):
        if hf_cache is None:
            raise ValueError(f"Offline inference needs --hf-cache containing {repo}")
        cache = Path(hf_cache).resolve() / ("models--" + repo.replace("/", "--"))
        if not cache.is_dir():
            raise ValueError(f"Missing offline model/tokenizer resource: {cache}")
        for path in cache.rglob("*"):
            if path.is_symlink() and (not path.exists() or not path.resolve().is_relative_to(cache.resolve())):
                raise ValueError(f"Broken or external cache link: {path}")
        revision = (cache / "refs/main").read_text().strip()
        for name in ("config.json", "tokenizer_config.json", "tokenizer.json", "preprocessor_config.json"):
            local_file(cache, f"snapshots/{revision}/{name}")
        caches.append(cache)
    output.parent.mkdir(parents=True, exist_ok=True)
    required = sum((checkpoint / n).stat().st_size for n in names)
    required += sum(p.stat().st_size for c in caches for p in c.rglob("*") if p.is_file() and not p.is_symlink())
    if shutil.disk_usage(output.parent).free < required + 64 * 1024 * 1024:
        raise ValueError(f"Need at least {required + 64 * 1024 * 1024} bytes free for a complete copy")
    temporary = Path(tempfile.mkdtemp(prefix=".model-bundle-", dir=output.parent))
    try:
        model = temporary / "model"
        model.mkdir()
        for name in sorted(names):
            target = model / name
            target.parent.mkdir(parents=True, exist_ok=True)
            copy_file(checkpoint / name, target)
        copy_file(Path(features), temporary / "features.json")
        for cache in caches:
            shutil.copytree(cache, temporary / "huggingface" / cache.name,
                            symlinks=True, copy_function=copy_file)
        copied = inspect_checkpoint(model)
        if copied["sha256"] != inspection["sha256"] or copied["optimizer_state_present"]:
            raise ValueError("Invalid extracted model")
        files = {}
        for path in sorted(temporary.rglob("*")):
            if path.is_symlink():
                files[str(path.relative_to(temporary))] = {"symlink": str(path.readlink()), "sha256": digest(path)}
            elif path.is_file():
                files[str(path.relative_to(temporary))] = {"bytes": path.stat().st_size, "sha256": digest(path)}
        full_size = sum(p.stat().st_size for p in checkpoint.parent.rglob("*") if p.is_file() and not p.is_symlink()) if checkpoint.name == "pretrained_model" else None
        manifest = {
            "format": "robot-model-bundle.v1", "label": label,
            "source_checkpoint": str(checkpoint), "source_model_sha256": inspection["sha256"],
            "source_optimizer_step": inspection["optimizer_step"], "model_type": config["type"],
            "model_directory": "model", "features": "features.json",
            "hf_hub_cache": "huggingface" if caches else None,
            "model_bytes": sum((model / n).stat().st_size for n in names),
            "bundle_bytes": sum(v.get("bytes", 0) for v in files.values()),
            "source_complete_checkpoint_bytes": full_size,
            "excluded": ["train_config.json", "training_state (optimizer, scheduler, RNG, step)"],
            "verification": "File hashes, tensor extents, feature shapes and processor references only; inference and task performance not tested by this export.",
            "files": files,
        }
        (temporary / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        (temporary / "README.md").write_text(
            f"# {label or config['type']} inference bundle\n\n"
            "Serve `model/` using `features.json` and an explicit device. "
            "Weights, configuration and normalization processors are unchanged copies. "
            "No optimizer state is included. Keep the original checkpoint to resume training.\n\n"
            + ("For offline use set `HF_HUB_CACHE` to the absolute path of this bundle's `huggingface/`. "
               "The workspace model wrapper selects this cache automatically with `--offline`.\n\n" if caches else "")
            + "See `manifest.json` for file hashes and source provenance. Export verifies file integrity, not learned task success.\n")
        os.replace(temporary, output)
        return {"output": str(output), **{k: manifest[k] for k in ("source_model_sha256", "source_optimizer_step", "model_bytes", "bundle_bytes", "source_complete_checkpoint_bytes")}}
    except BaseException:
        shutil.rmtree(temporary)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--features", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hf-cache", type=Path)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--label")
    args = parser.parse_args()
    print(json.dumps(package(**vars(args)), indent=2))


if __name__ == "__main__":
    main()
