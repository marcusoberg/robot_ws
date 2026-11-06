from pathlib import Path
import json
import time
import threading
from scservo_sdk import COMM_RX_TIMEOUT, PacketHandler, PortHandler

from robot_teleop.config import teleop_settings
from robot_core.validation import leader_calibration


class LeaderFeedbackTimeout(TimeoutError):
    pass


class SerialLeader:
    def __init__(self, port, calibration_path, spec):
        self.calibration = json.loads(Path(calibration_path).read_text())
        leader_calibration(self.calibration, spec.joints)
        self.spec = spec
        self.reader = None
        self.reader_stop = threading.Event()
        self.reader_lock = threading.Lock()
        self.latest_sample = None
        self.reader_error = None
        self.port = PortHandler(port)
        if not self.port.openPort() or not self.port.setBaudRate(teleop_settings()['hardware']['leader_baud']):
            self.port.closePort()
            raise RuntimeError(f"Cannot open leader {port}")
        self.bus = PacketHandler(0)
        try:
            for name in spec.joints:
                c = self.calibration[name]
                offset, result, error = self.bus.read2ByteTxRx(self.port, c["id"], 31)
                if result or (error & ~0x01):
                    raise RuntimeError(f"Leader homing offset read failed: {name} ({result}, {error})")
                signed_offset = -(offset & 2047) if offset & 2048 else offset
                if signed_offset != c["homing_offset"]:
                    raise ValueError(f"Leader homing offset differs from calibration: {name}; "
                                     f"motor={signed_offset}, file={c['homing_offset']}")
                result, error = self.bus.write1ByteTxRx(self.port, c["id"], 40, 0)
                if result or (error & ~0x01):
                    raise RuntimeError(f"Cannot disable leader torque: {name}")
        except BaseException:
            self.close(); raise

    def start_reader(self):
        def acquire():
            period = self.spec.period_ns / 1e9
            while not self.reader_stop.is_set():
                started = time.monotonic()
                try:
                    sample = self.read_raw()
                    with self.reader_lock:
                        self.latest_sample = sample
                        self.reader_error = None
                except Exception as error:
                    with self.reader_lock:
                        self.reader_error = error
                self.reader_stop.wait(max(0., period - (time.monotonic() - started)))

        self.reader = threading.Thread(target=acquire, name='so101-feedback', daemon=True)
        self.reader.start()

    def latest_raw(self):
        """Read the acquisition snapshot without blocking on serial transactions."""
        with self.reader_lock:
            if self.reader_error is not None:
                raise self.reader_error
            if self.latest_sample is None:
                raise LeaderFeedbackTimeout('Waiting for first leader sample')
            return self.latest_sample

    def read_raw(self):
        acquired = time.monotonic_ns()
        raw = {}
        for name in self.spec.joints:
            value, result, error = self.bus.read2ByteTxRx(self.port, self.calibration[name]["id"], 56)
            if result == COMM_RX_TIMEOUT:
                raise LeaderFeedbackTimeout(f"Leader feedback timed out: {name} (COMM_RX_TIMEOUT, {result})")
            if result or (error & ~0x01):
                raise RuntimeError(f"Leader feedback failed: {name} ({result}, {error})")
            raw[name] = value
        return raw, acquired

    def close(self):
        self.reader_stop.set()
        if self.reader is not None:
            self.reader.join()
        self.port.closePort()
