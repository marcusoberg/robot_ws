"""Read-only integrity checks. No robot/model process or simulation is started."""

from pathlib import Path
import argparse
import hashlib
import json
import os
import time

ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = ROOT / "artifacts/legacy-20261011"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def snapshot(directory):
    rows = []
    for parent, dirs, files in os.walk(directory, followlinks=False):
        for name in sorted(dirs + files):
            path = Path(parent) / name
            relative = str(path.relative_to(directory))
            if path.is_symlink():
                rows.append(
                    {
                        "path": relative,
                        "link": os.readlink(path),
                        "missing": not path.exists(),
                    }
                )
            elif path.is_file():
                rows.append(
                    {
                        "path": relative,
                        "size": path.stat().st_size,
                        "sha256": digest(path),
                    }
                )
    return sorted(rows, key=lambda row: row["path"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifacts",
        action="store_true",
        help="Rehash all preserved artifacts (approximately 24 GB)",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    expected = json.loads((ARTIFACTS / "parol6_demo-before.json").read_text())
    actual = snapshot(ROOT / "src/parol6_demo")
    old = {row["path"]: row for row in expected}
    new = {row["path"]: row for row in actual}
    changed = [
        name
        for name in sorted(old.keys() | new.keys())
        if old.get(name) != new.get(name)
    ]
    inventory = json.loads((ARTIFACTS / "inventory.json").read_text())
    finder_changes = [name for name in changed if Path(name).name == ".DS_Store"]
    content_changes = [name for name in changed if name not in finder_changes]
    errors = []
    if args.artifacts:
        for row in inventory["files"]:
            path = Path(row["preserved"])
            wanted = row.get("materialized_sha256") or row.get("sha256")
            if wanted:
                if not path.is_file() or digest(path) != wanted:
                    errors.append(str(path))
            elif not path.is_symlink():
                errors.append(str(path) + " missing symlink")
            elif row.get("missing") and path.exists():
                errors.append(str(path) + " expected missing link changed")
    report = {
        "schema": "workspace.integrity.v1",
        "checked_unix": time.time(),
        "original_runtime_unchanged": not changed,
        "original_runtime_entries": len(actual),
        "original_source_git_and_existing_generated_files_unchanged": not content_changes,
        "finder_metadata_changes": finder_changes,
        "changed_original_paths": changed,
        "artifact_preservation_entries": len(inventory["files"]),
        "artifact_copy_hashes_verified_before_archive": (
            ROOT / ".workspace/artifacts-verified"
        ).is_file(),
        "artifacts_rehashed_now": args.artifacts,
        "artifact_errors": errors,
        "missing_original_smolvla_links": len(inventory["missing"]),
        "archived_repositories": {
            name: (ROOT / "archive" / name / ".git").is_dir()
            for name in (
                "policy_runner",
                "robot-ros",
                "robot-data",
                "robot-experiments",
            )
        },
        "new_workspace_git_created": (ROOT / ".git").exists(),
        "operational_smoke_run": False,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return bool(
        content_changes
        or errors
        or report["new_workspace_git_created"]
        or not all(report["archived_repositories"].values())
    )


if __name__ == "__main__":
    raise SystemExit(main())
