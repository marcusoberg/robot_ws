import threading

class ObservationWorker:
    def __init__(self, env):
        self.env=env;self.latest=None;self.failure=None;self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    def _run(self):
        previous=None
        try:
            while not self.stop.is_set():
                snap=self.env.native.latest_boundary()
                key=(snap.episode,snap.time)
                if key!=previous:
                    self.latest=self.env.observation(snap);previous=key
                self.stop.wait(.005)
        except Exception as exc:self.failure=str(exc)
    def close(self):
        self.stop.set();self.thread.join()

