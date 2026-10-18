from pathlib import Path
import json
import math
import time
import numpy as np


def validate_calibration(calibration, joints):
    if set(calibration) != set(joints):
        raise ValueError("Calibration must define exactly the leader's ordered joints")
    low, midpoints, high = [], [], []
    for i, name in enumerate(joints):
        c = calibration[name]
        required = {"range_min", "range_max", "id", "drive_mode", "homing_offset"}
        if not isinstance(c, dict) or not required.issubset(c):
            raise ValueError(f"Incomplete calibration for {name}")
        if any(type(c[key]) not in (int, float) for key in ("range_min", "range_max")):
            raise ValueError(f"Calibration ranges must be numeric: {name}")
        lo, hi = c["range_min"], c["range_max"]
        mid = c.get("neutral", (lo + hi) / 2)
        if (type(mid) not in (int, float) or type(c["id"]) is not int or type(c["homing_offset"]) is not int
                or c["id"] != i + 1 or c["drive_mode"] != 0 or not -2047 <= c["homing_offset"] <= 2047
                or not all(math.isfinite(v) for v in (lo, mid, hi)) or not 0 <= lo < mid < hi <= 4095):
            raise ValueError(f"Invalid calibration for {name}")
        low.append(lo); midpoints.append(mid); high.append(hi)
    return low, midpoints, high


class LeaderMapping:
    def __init__(self, spec, calibration):
        self.spec = spec
        self.low, self.mid, self.high = validate_calibration(calibration, spec.joints)
        for i, name in enumerate(spec.joints):
            if not spec.lower[i] < 0 < spec.upper[i]:
                raise ValueError(f"Joint mapping requires a zero neutral inside the bounds: {name}")
        self.neutral = np.zeros(len(spec.joints))
        self.neutral[-1] = (spec.lower[-1] + spec.upper[-1]) / 2
        for values in (self.low, self.mid, self.high):
            self.map(dict(zip(spec.joints, values)))

    def map(self, raw):
        if set(raw) != set(self.spec.joints):
            raise ValueError("Leader sample joint names differ from calibration")
        q = []
        for i, name in enumerate(self.spec.joints):
            x = raw[name]
            if not math.isfinite(x) or not self.low[i] <= x <= self.high[i]:
                raise ValueError(f"Leader {name} sample {x} outside calibrated [{self.low[i]}, {self.high[i]}]")
            if x <= self.mid[i]:
                a, b, lo, hi = self.low[i], self.mid[i], self.spec.lower[i], self.neutral[i]
            else:
                a, b, lo, hi = self.mid[i], self.high[i], self.neutral[i], self.spec.upper[i]
            q.append(lo if x == a else hi if x == b else lo + (x-a)/(b-a)*(hi-lo))
        return self.spec.validate(q)

class SerialLeader:
    def __init__(self, port, calibration_path, spec, *, map_positions=True):
        from scservo_sdk import PortHandler, PacketHandler
        if not port:
            raise ValueError("An explicit serial port is required")
        self.calibration = json.loads(Path(calibration_path).read_text())
        validate_calibration(self.calibration, spec.joints)
        self.mapping = LeaderMapping(spec, self.calibration) if map_positions else None
        self.spec = spec
        self.port = PortHandler(port)
        if not self.port.openPort() or not self.port.setBaudRate(1_000_000):
            self.port.closePort()
            raise RuntimeError(f"Cannot open leader {port}")
        self.bus = PacketHandler(0)
        try:
            for name in spec.joints:
                c = self.calibration[name]
                offset, result, error = self.bus.read2ByteTxRx(self.port, c["id"], 31)
                signed_offset = -(offset & 2047) if offset & 2048 else offset
                if result or error or signed_offset != c["homing_offset"]:
                    raise ValueError(f"Leader motor calibration differs from file: {name}; calibrate before connecting")
                for key,address in (("range_min",9),("range_max",11)):
                    value,result,error=self.bus.read2ByteTxRx(self.port,c["id"],address)
                    if result or error or value!=c[key]:
                        raise ValueError(f"Leader motor {key} differs from calibration: {name}")
                result, error = self.bus.write1ByteTxRx(self.port, c["id"], 40, 0)
                if result or error:
                    raise RuntimeError(f"Cannot disable leader torque: {name}")
        except BaseException:
            self.close(); raise

    def read(self):
        if self.mapping is None:
            raise RuntimeError('This leader is configured for raw acquisition; use read_raw()')
        raw, acquired = self.read_raw()
        return self.mapping.map(raw), acquired

    def read_raw(self):
        acquired = time.monotonic_ns()
        raw = {}
        for name in self.spec.joints:
            value, result, error = self.bus.read2ByteTxRx(self.port, self.calibration[name]["id"], 56)
            if result or (error & ~0x01):
                raise RuntimeError(f"Leader feedback failed: {name} ({result}, {error})")
            raw[name] = value
        return raw, acquired

    def close(self):
        self.port.closePort()
