from pathlib import Path
import json
import queue
import threading
import uuid
import numpy as np

class Recorder:
    def __init__(self, root, spec, *, purpose="demonstrations"):
        self.root, self.spec, self.purpose = Path(root), spec, purpose
        self.active = None
        self.frames = 0
        self.failure = None
        self.jobs = queue.Queue(maxsize=32)
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def _write(self):
        while True:
            job = self.jobs.get()
            try:
                if job is None:
                    return
                path, arrays, metadata = job
                np.savez_compressed(path.with_suffix(".npz"), **arrays)
                path.with_suffix(".json").write_text(json.dumps(metadata, allow_nan=False))
            except Exception as exc:
                self.failure = str(exc)
            finally:
                self.jobs.task_done()

    def start(self, observation):
        if self.active:
            raise ValueError("An episode is already recording")
        if observation.time_ns != 0:
            raise ValueError("Reset before recording; start recording before activating motion")
        self.next_time = observation.time_ns
        self.active = self.root / "pending" / uuid.uuid4().hex
        self.active.mkdir(parents=True, exist_ok=False)
        self.identity = (observation.instance, observation.episode)
        self.frames, self.failure = 0, None
        (self.active / "action_spec.json").write_text(self.spec.to_json())
        self._manifest("incomplete", "recording")
        return str(self.active)

    def _manifest(self, status, reason, success=False):
        value = dict(format="robot-episode.v2", status=status, reason=reason, success=success,
                     purpose=self.purpose, frames=self.frames, identity=self.identity, action_spec=self.spec.identity)
        (self.active / "episode.json").write_text(json.dumps(value, indent=2))

    def append(self, before, action, after, snapshot):
        if not self.active:
            return
        if self.failure:
            raise RuntimeError(f"Recording failed: {self.failure}")
        q = self.spec.validate(action)
        if (self.identity != (before.instance, before.episode) or before.episode != after.episode
            or before.instance != after.instance or snapshot.episode != before.episode or before.time_ns != self.next_time
            or before.time_ns + self.spec.period_ns != after.time_ns
            or snapshot.time != after.time_ns or snapshot.status.mode != "Running"
            or not snapshot.trace or snapshot.trace[0].begin != before.time_ns
            or snapshot.trace[-1].end != after.time_ns
            or any(not np.array_equal(t.reference, q) or t.sequence != snapshot.status.delivered_sequence for t in snapshot.trace)
            or set(before.images) != set(self.spec.cameras)):
            raise ValueError("Recording does not match one successfully installed action interval")
        arrays = {"observation.state": before.position, "observation.velocity": before.velocity,
                  "action": q, "subsequent.state": after.position,
                  **{f"observation.images.camera_{k}": v for k,v in before.images.items()}}
        for camera in self.spec.cameras:
            if arrays[f"observation.images.camera_{camera}"].shape != self.spec.image_shape:
                raise ValueError("Recording camera shape disagrees with action specification")
        meta = dict(index=self.frames, episode=before.episode, snapshot=before.snapshot,
                    acquired_ns=before.time_ns, applied_ns=snapshot.trace[0].begin,
                    host_acquired_ns=before.host_acquired_ns, action_spec=self.spec.identity, cameras=before.camera_metadata,
                    trace=[dict(begin=t.begin,end=t.end,reference=t.reference,measured=t.measured,sequence=t.sequence) for t in snapshot.trace])
        try:
            self.jobs.put_nowait((self.active / f"frame-{self.frames:06}", arrays, meta))
        except queue.Full:
            self.failure = "recording writer queue exhausted"
            raise RuntimeError(self.failure)
        self.frames += 1
        self.next_time = after.time_ns

    def finish(self, *, complete=False, success=False, reason="operator_save"):
        if not self.active:
            raise ValueError("No episode is recording")
        self.jobs.join()
        valid = complete and self.frames > 0 and self.failure is None
        status = "complete" if valid else "incomplete"
        self._manifest(status, self.failure or reason, success if valid else False)
        destination = self.root / ("complete" if valid else "interrupted") / self.active.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        self.active.rename(destination)
        self.active = None
        return str(destination)

    def interrupt(self, reason):
        if self.active:
            return self.finish(reason=reason)

    def close(self):
        self.interrupt("application_closed")
        self.jobs.put(None)
        self.worker.join()
