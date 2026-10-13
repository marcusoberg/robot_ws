#!/usr/bin/env bash
set -euo pipefail
runtime_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
native_dir="${ROBOT_RUNTIME_NATIVE_ENV:-$runtime_dir/.dependencies/native}"
python_bin="${ROBOT_PYTHON:-python3.12}"
if [[ ! -x "$native_dir/bin/python" ]]; then "$python_bin" -m venv "$native_dir"; fi
"$native_dir/bin/python" -m pip install -r "$runtime_dir/scripts/native-requirements.txt"
