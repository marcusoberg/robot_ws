"""Training and serving commands; settings come from environment, then CLI overrides."""
import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = ROOT / "environments/models"
PYTHON = ENVIRONMENT / ".venv/bin/python"

def run(command, **kwargs):
    print(shlex.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, **kwargs)

def environment():
    # Model processes must not import ROS/Pixi libraries into the isolated Torch environment.
    env = dict(os.environ)
    for name in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH", "DYLD_FALLBACK_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        env.pop(name, None)
    env["PYTHONNOUSERSITE"] = "1"
    return env

def require(value, name):
    if not value:
        raise ValueError(f"Set {name} or pass its command-line option")
    return value

def label(value):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", value or ""):
        raise ValueError("Run/model name must use letters, digits, dots, underscores or hyphens")
    return value

def remote(a):
    host = require(a.host, "REMOTE_HOST")
    user = require(a.user, "REMOTE_USER")
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", host) or host.startswith("-") or not re.fullmatch(r"[A-Za-z0-9_.-]+", user) or user.startswith("-"):
        raise ValueError("Invalid SSH host/user")
    if not 1 <= a.port <= 65535:
        raise ValueError("Invalid SSH port")
    directory = PurePosixPath(require(a.remote_dir, "REMOTE_DIR"))
    if not directory.is_absolute() or ".." in directory.parts:
        raise ValueError("REMOTE_DIR must be an absolute remote workspace path")
    return ["ssh", "-p", str(a.port), f"{user}@{host}"], str(directory)

def dispatch(a, args, *, background=None):
    ssh, directory = remote(a)
    command = f"cd {shlex.quote(directory)} && " + shlex.join(["python3", "scripts/manage.py", *args])
    if background:
        log = f"{directory}/training/runs/{label(a.run)}/{background}.log"
        session = f"robot-{background}-{a.run}"
        command = (f"mkdir -p {shlex.quote(str(PurePosixPath(log).parent))} && "
                   + shlex.join(["tmux", "new-session", "-d", "-s", session,
                                 command + " > " + shlex.quote(log) + " 2>&1"]))
        print(f"Remote session: {session}; log: {log}", flush=True)
    if a.command == "serve":
        ssh = [*ssh[:-1], "-tt", ssh[-1]]
    run([*ssh, command])

def configuration(a):
    return require(a.config, "CONFIG")

def train(a):
    require(a.device, "DEVICE")
    label(a.run)
    config = configuration(a)
    if a.remote:
        args = ["train", "--run", a.run, "--device", a.device, "--config", config]
        for name in ("dataset", "dataset_root", "steps", "batch_size"):
            if getattr(a, name) is not None:
                args += ["--"+name.replace("_", "-"), str(getattr(a, name))]
        if a.resume:
            args.append("--resume")
        dispatch(a, args, background="train")
        return
    config_path = Path(config)
    if not config_path.is_absolute():
        config_path = ROOT / config_path
    if not config_path.is_file():
        raise ValueError(f"Config does not exist: {config_path}")
    if not a.resume and not a.dataset:
        raise ValueError("Set DATASET_REPO_ID for this training run; no dataset is chosen implicitly")
    output = ROOT / "training/runs" / a.run / "output"
    if output.exists() and not a.resume:
        raise ValueError(f"Run output exists: {output}; choose another RUN")
    command = [PYTHON, "-m", "lerobot.scripts.lerobot_train", f"--config_path={config_path}",
               f"--output_dir={output}", f"--policy.device={a.device}",
               "--policy.push_to_hub=false", "--save_checkpoint_to_hub=false"]
    if a.resume:
        # LeRobot resolves full optimizer/scheduler/RNG state relative to train_config.json.
        if config_path.name != "train_config.json" or not (config_path.parent.parent / "training_state").is_dir():
            raise ValueError("Resume requires a complete checkpoint's pretrained_model/train_config.json")
        command.append("--resume=true")
        if a.device == "mps":
            print("MPS resume does not guarantee exact RNG replay.", flush=True)
    for flag, value in (("dataset.repo_id", a.dataset), ("dataset.root", a.dataset_root),
                        ("steps", a.steps), ("batch_size", a.batch_size)):
        if value is not None:
            command.append(f"--{flag}={value}")
    model_env = environment()
    model_env["ACCELERATE_TORCH_DEVICE"] = a.device
    # LeRobot may otherwise silently fall back from a requested unavailable device.
    run([PYTHON, "-c", "import sys, torch; d=sys.argv[1]; "
         "ok=d=='cpu' or (d=='cuda' and torch.cuda.is_available()) or (d=='mps' and torch.backends.mps.is_available()); "
         "sys.exit(0 if ok else 'Requested training device is unavailable: '+d)", a.device], env=model_env)
    output.parent.mkdir(parents=True, exist_ok=True)
    (output.parent / "command.json").write_text(json.dumps(list(map(str, command)), indent=2)+"\n")
    run(command, cwd=ROOT, env=model_env)

def serve(a):
    require(a.device, "DEVICE")
    model = label(require(a.model, "MODEL"))
    if a.remote:
        args = ["serve", "--model", model, "--device", a.device, "--listen-port", str(a.listen_port)]
        if a.offline:
            args.append("--offline")
        # Serving is foreground; Ctrl-C terminates this SSH-owned server.
        dispatch(a, args)
        return
    command = [PYTHON, "-m", "robot_inference.server", "--bundle", ROOT/"models"/model,
               "--device", a.device, "--port", a.listen_port]
    if a.offline:
        command.append("--offline")
    run(command, cwd=ROOT, env=environment())

def export(a):
    model = label(require(a.model, "MODEL"))
    checkpoint = require(a.checkpoint, "CHECKPOINT")
    deployment = require(a.deployment, "DEPLOYMENT")
    if a.remote:
        args = ["export", "--model", model, "--checkpoint", checkpoint, "--deployment", deployment]
        if a.hf_cache:
            args += ["--hf-cache", a.hf_cache]
        dispatch(a, args)
        return
    command = [PYTHON, "-m", "robot_inference.bundle", "--checkpoint", checkpoint,
               "--deployment", deployment, "--output", ROOT/"models"/model]
    if a.hf_cache:
        command += ["--hf-cache", a.hf_cache]
    run(command, cwd=ROOT, env=environment())

def sync(a):
    ssh, directory = remote(a)
    endpoint = ssh[-1]
    version = subprocess.check_output(["rsync", "--version"], text=True)
    if not re.search(r"rsync\s+version\s+[3-9]\.", version):
        raise ValueError("rsync 3 is required. Run inside the workspace Pixi shell; install rsync 3 remotely too.")
    base = ["rsync", "-az", "--protect-args", "-e", shlex.join(ssh[:-1])]
    if a.dry_run:
        base.append("--dry-run")
    for pattern in a.include:
        base += ["--include", pattern]
    for pattern in a.exclude:
        base += ["--exclude", pattern]
    base += ["--exclude", ".venv/", "--exclude", "__pycache__/", "--exclude", ".env",
             "--exclude", ".git/"]
    if a.up:
        if not a.dry_run:
            run([*ssh, "mkdir -p " + shlex.quote(directory)])
        if a.path:
            source = (ROOT / a.path).resolve()
            if not source.is_relative_to(ROOT) or not source.exists():
                raise ValueError("--path must exist inside this workspace")
            relative = str(source.relative_to(ROOT))
            run([*base, "--relative", str(ROOT)+"/./"+relative, f"{endpoint}:{directory}/"])
        else:
            # Explicit code allowlist; no credentials, checkpoints or datasets uploaded implicitly.
            run([*base, "--exclude", "runs/", "--relative",
                 *[str(ROOT)+"/./"+p for p in ("training", "inference", "scripts/manage.py",
                    "environments/models/pyproject.toml", "environments/models/uv.lock")],
                 f"{endpoint}:{directory}/"])
    else:
        if bool(a.run) == bool(a.model):
            raise ValueError("Download exactly one --run or --model")
        relative = "models/"+label(a.model) if a.model else "training/runs/"+label(a.run)
        if a.path:
            raise ValueError("--path is upload-only; download with --run/--model and optional --checkpoint")
        if a.checkpoint:
            if not a.run:
                raise ValueError("--checkpoint requires --run")
            relative += "/output/checkpoints/" + label(a.checkpoint)
        if a.run and not a.checkpoints and not a.checkpoint:
            base += ["--exclude", "checkpoints/", "--exclude", "*.safetensors", "--exclude", "*.pt"]
        destination = ROOT / relative
        if not a.dry_run:
            destination.mkdir(parents=True, exist_ok=True)
        run([*base, f"{endpoint}:{directory}/{relative}/", str(destination)+"/"])

def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    for name in ("setup", "train", "serve", "export", "sync", "status", "logs", "stop", "tunnel"):
        q = sub.add_parser(name)
        q.add_argument("--remote", action="store_true")
        for key, env, default in (("host","REMOTE_HOST",None),("user","REMOTE_USER",None),
                ("remote-dir","REMOTE_DIR",None),("run","RUN",None),("model","MODEL",None)):
            q.add_argument("--"+key, default=os.getenv(env, default))
        q.add_argument("--port", type=int, default=int(os.getenv("REMOTE_PORT", "22")))
        if name in ("train", "serve"):
            q.add_argument("--device", choices=("cpu","cuda","mps"), default=os.getenv("DEVICE"))
        if name == "train":
            q.add_argument("--config", default=os.getenv("CONFIG", "training/configs/act.yaml"))
            q.add_argument("--dataset", default=os.getenv("DATASET_REPO_ID"))
            q.add_argument("--dataset-root", default=os.getenv("DATASET_ROOT"))
            q.add_argument("--steps", type=int, default=os.getenv("STEPS"))
            q.add_argument("--batch-size", type=int, default=os.getenv("BATCH_SIZE"))
            q.add_argument("--resume", action="store_true")
        if name in ("serve", "tunnel"):
            q.add_argument("--listen-port", type=int, default=int(os.getenv("MODEL_PORT","8000")))
        if name == "serve":
            q.add_argument("--offline", action="store_true", default=os.getenv("OFFLINE","0") == "1")
        if name == "export":
            q.add_argument("--checkpoint", default=os.getenv("CHECKPOINT"))
            q.add_argument("--deployment", default=os.getenv("DEPLOYMENT"))
            q.add_argument("--hf-cache", default=os.getenv("HF_HUB_CACHE"))
        if name == "sync":
            direction = q.add_mutually_exclusive_group(required=True)
            direction.add_argument("--up", action="store_true")
            direction.add_argument("--down", action="store_true")
            q.add_argument("--path", help="Upload a selected workspace path instead of code")
            q.add_argument("--checkpoint", help="Download one complete checkpoint from the selected run")
            q.add_argument("--checkpoints", action="store_true", help="Include full training checkpoints when downloading a run")
            q.add_argument("--include", action="append", default=[])
            q.add_argument("--exclude", action="append", default=[])
            q.add_argument("--dry-run", action="store_true")
    a = p.parse_args()
    try:
        if a.command in ("train","serve","export","sync"):
            globals()[a.command](a)
        elif a.command == "setup":
            if a.remote:
                dispatch(a, ["setup"])
            else:
                run(["uv", "sync", "--locked", "--project", ENVIRONMENT, "--python", "3.12"], env=environment())
        elif a.command == "tunnel":
            ssh, _ = remote(a)
            run([*ssh[:-1], "-N", "-o", "ExitOnForwardFailure=yes", "-L",
                 f"127.0.0.1:{a.listen_port}:127.0.0.1:{a.listen_port}", ssh[-1]])
        elif a.command == "status":
            ssh, _ = remote(a)
            run([*ssh, "tmux list-sessions"])
        elif a.command == "stop":
            ssh, _ = remote(a)
            run([*ssh, shlex.join(["tmux", "send-keys", "-t", f"robot-train-{label(a.run)}", "C-c"])])
        elif a.command == "logs":
            ssh, directory = remote(a)
            run([*ssh, "tail -f " + shlex.quote(f"{directory}/training/runs/{label(a.run)}/train.log")])
    except KeyboardInterrupt:
        p.exit(130)
    except (ValueError, FileNotFoundError, subprocess.CalledProcessError) as error:
        p.exit(1, str(error)+"\n")

if __name__ == "__main__":
    main()
