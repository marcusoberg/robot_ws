"""Bounded in-memory command history, written only after motion has faulted."""

from collections import deque
import json
from pathlib import Path
import tempfile
import time


class MotionTrace:
    def __init__(self, label, joints):
        self.label = label
        self.joints = list(joints)
        self.records = deque(maxlen=4096)
        self.path = None

    def reset(self):
        self.records.clear()
        self.path = None

    def record(self, kind, **values):
        self.records.append(dict(monotonic_ns=time.monotonic_ns(), kind=kind, **values))

    def save(self, reason):
        if self.path is not None or not self.records:
            return self.path
        # No file IO in the moving command loop. Preserve each fault separately.
        with tempfile.NamedTemporaryFile(mode='w', prefix=f'parol6-{self.label}-',
                                         suffix='.json', dir='/private/tmp', delete=False) as output:
            json.dump(dict(reason=reason, joints=self.joints,
                           feedback='firmware step counters, not encoders',
                           events=list(self.records)), output)
            self.path = str(Path(output.name).resolve())
        return self.path
