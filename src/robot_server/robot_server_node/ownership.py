"""Local ownership of the global /robot topics, shared with ROS launch."""

import fcntl
import json
import os
import signal
from pathlib import Path
import tempfile
import time


def lock_path():
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    return Path(tempfile.gettempdir()) / f"robot-server-{os.getuid()}-{domain}.lock"


def acquire_server_lock():
    lock = lock_path().open("a+")
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(
            "Another robot server already owns this ROS domain"
        ) from None
    return lock


def existing_server():
    """Stale files are harmless: the OS lock, not file existence, proves ownership."""
    try:
        lock = acquire_server_lock()
    except RuntimeError:
        return json.loads(lock_path().read_text())
    else:
        lock.close()
        return None


def describe_owner(lock, **configuration):
    lock.seek(0)
    lock.truncate()
    json.dump({"pid": os.getpid(), **configuration}, lock)
    lock.flush()


def stop_existing_server(timeout=10.0):
    """Interrupt this domain's owner and wait for backend cleanup to release its lock."""
    owner = existing_server()
    if owner is None:
        return False
    pid = owner.get("pid")
    if type(pid) is not int or pid <= 1:
        raise RuntimeError("Running server has no valid owner PID in its lock metadata")
    if existing_server() != owner:
        raise RuntimeError("Server owner changed during shutdown; retry the command")
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + timeout
    while True:
        current = existing_server()
        if current is None:
            return True
        if current != owner:
            raise RuntimeError("Another server acquired ownership during shutdown")
        if time.monotonic() >= deadline:
            raise TimeoutError("Server did not finish shutting down within 10 seconds")
        time.sleep(0.05)
