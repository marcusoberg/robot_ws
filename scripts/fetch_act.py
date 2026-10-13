"""Fetch the pinned public ACT inference files; no Torch, ROS or credentials needed."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
from urllib.error import URLError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "environments/models/act-public.json"


def verify(path, expected):
    if not path.is_file() or path.stat().st_size != expected["bytes"]:
        return False
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest() == expected["sha256"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "models/act-base/model")
    parser.add_argument("--check", action="store_true", help="Verify existing files without downloading")
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    output = args.output.resolve()
    try:
        if not args.check:
            output.mkdir(parents=True, exist_ok=True)
        for name, expected in manifest["files"].items():
            if Path(name).name != name or name in (".", ".."):
                raise ValueError("Manifest filenames must be simple basenames")
            target = output / name
            if verify(target, expected):
                print(f"Verified {name}", flush=True)
                continue
            if args.check or target.exists() or target.is_symlink():
                raise ValueError(f"Missing or different file: {target}. Choose a new --output directory to fetch a fresh copy.")
            url = f"https://huggingface.co/{manifest['repository']}/resolve/{manifest['revision']}/{name}"
            print(f"Downloading {name} ({expected['bytes'] / 1_000_000:.1f} MB)", flush=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=output, prefix=f".{name}.", suffix=".part", delete=False) as stream:
                    temporary = Path(stream.name)
                    count = 0
                    with urlopen(url, timeout=60) as response:
                        while block := response.read(1024 * 1024):
                            count += len(block)
                            if count > expected["bytes"]:
                                raise ValueError(f"Unexpected download size for {name}")
                            stream.write(block)
                if not verify(temporary, expected):
                    raise ValueError(f"Size or SHA-256 mismatch for {name}")
                os.replace(temporary, target)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        print(f"ACT ready: {output}\nRevision: {manifest['revision']}\nNo model or robot process was started.")
    except (OSError, URLError, ValueError) as error:
        parser.exit(1, f"ACT download/verification failed: {error}\n")


if __name__ == "__main__":
    main()
