#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$project_dir"
local_pixi=false
case "${1:-}" in
    "") ;;
    --local-pixi) local_pixi=true; shift ;;
    --help|-h)
        printf 'Usage: ./scripts/setup.sh [--local-pixi]\nInstall the project; --local-pixi keeps Pixi in .tools/.\n'
        exit 0 ;;
    *) printf 'Unknown option: %s\nUse ./scripts/setup.sh --help\n' "$1" >&2; exit 2 ;;
esac
if [ "$#" -ne 0 ]; then
    printf 'Use ./scripts/setup.sh [--local-pixi]\n' >&2
    exit 2
fi
if [ "$local_pixi" = true ] || [ -x .tools/pixi ] || ! command -v pixi >/dev/null 2>&1; then
    if [ ! -x .tools/pixi ]; then
        case "$(uname -s)-$(uname -m)" in
            Darwin-arm64) platform=aarch64-apple-darwin ;;
            Linux-x86_64) platform=x86_64-unknown-linux-musl ;;
            *) printf 'Supported computers: Apple Silicon macOS and Linux x86_64\n' >&2; exit 1 ;;
        esac
        mkdir -p .tools
        curl --fail --location "https://github.com/prefix-dev/pixi/releases/download/v0.81.0/pixi-${platform}.tar.gz" -o .tools/pixi.tar.gz
        tar -xzf .tools/pixi.tar.gz -C .tools
    fi
    pixi_command="$project_dir/.tools/pixi"
else
    pixi_command="$(command -v pixi)"
fi
printf 'Using Pixi: %s\n' "$pixi_command"
"$pixi_command" install --locked
"$pixi_command" run setup
printf '\nSetup complete. Activate the environment from ws:\n  %s shell\nThen check installation:\n  pixi run doctor\n' "$pixi_command"
