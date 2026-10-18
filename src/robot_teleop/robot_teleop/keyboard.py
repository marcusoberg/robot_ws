import os
import select
import termios
import time
import tty


class Keyboard:
    def __init__(self, node, command, enabled=True):
        self.fd = None
        self.saved = None
        self.timer = None
        self.last_space = 0.
        self.command = command
        if not enabled:
            return
        try:
            self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NONBLOCK)
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.timer = node.create_timer(.02, self.poll)
        except OSError as exc:
            self.close()
            node.get_logger().warning(f'No controlling terminal; use pixi run engage/disengage: {exc}')

    def poll(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return
        for key in os.read(self.fd, 128):
            operation = {32: 'toggle', ord('r'): 'record', ord('s'): 'save', ord('d'): 'discard', ord('x'): 'reset'}.get(key)
            if operation == 'toggle':
                now = time.monotonic()
                if now - self.last_space < .6:
                    continue
                self.last_space = now
            if operation:
                self.command(operation)

    def close(self):
        if self.timer:
            self.timer.cancel()
        if self.fd is not None:
            try:
                if self.saved is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            finally:
                os.close(self.fd)
                self.fd = None
