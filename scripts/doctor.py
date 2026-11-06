"""Report installed dependencies without opening a robot connection."""

from importlib.metadata import version
import json

print(json.dumps({name: version(name) for name in ('parol6', 'robot-core', 'pin', 'numpy')}, indent=2))
