"""Read Space from the controlling terminal even when ros2 launch owns stdin."""
import os
import select
import termios
import time
import tty


class SpaceKey:
    def __init__(self, node, toggle, enabled=True):
        self.fd = None
        self.saved = None
        self.timer = None
        self.last_press = 0.0
        self.toggle = toggle
        if not enabled:
            return
        try:
            self.fd = os.open("/dev/tty", os.O_RDWR | os.O_NONBLOCK)
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.timer = node.create_timer(0.02, self.poll)
        except OSError as error:
            self.close()
            node.get_logger().warning("No interactive terminal: use /teleop/engage instead. " + str(error))

    def poll(self):
        if self.fd is not None and select.select([self.fd], [], [], 0)[0]:
            keys = os.read(self.fd, 128)
            now = time.monotonic()
            if b" " in keys:
                previous, self.last_press = self.last_press, now
                if now-previous > 0.6:
                    self.toggle()

    def close(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.fd is not None:
            try:
                if self.saved is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            finally:
                os.close(self.fd)
                self.fd = None
