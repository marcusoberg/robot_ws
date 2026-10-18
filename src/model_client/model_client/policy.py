from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import time
import uuid
import numpy as np
from robot_inference.contracts import Features, Identity, PolicyInput, validate_input, validate_prediction

@dataclass(frozen=True)
class ActionSample:
    goal: tuple[float, ...]
    observation: Identity
    action_index: int
    time_ns: int

class PolicyRunner:
    def __init__(self, spec, client):
        self.spec, self.client = spec, client
        self.features = Features(spec.to_json())
        if client.features != self.features:
            raise ValueError("Inference client action contract differs")
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="inference")
        self.pending = None
        self.latencies_ns = []
        self.diagnostics = []
        self.reset()

    def reset(self):
        self.context = uuid.uuid4().hex
        self.buffer = {}
        self.episode = None
        self.last_request = -5
        if self.pending:
            self.pending[0].cancel()
        # Keep the single outstanding request until it finishes; its context makes it unusable.

    def _input(self, obs):
        meta = {"images": {f"observation.images.camera_{k}": {**obs.camera_metadata.get(k,{}), "stamp": {
            "sequence": obs.snapshot, "epoch": obs.episode, "source_clock": "simulation",
            "source_time_ns": obs.time_ns, "host_received_ns": obs.host_acquired_ns}} for k in obs.images}}
        return validate_input(PolicyInput(Identity(obs.instance,obs.episode,obs.snapshot),obs.time_ns,
            obs.features(),"place the cube inside the container",str(obs.episode),self.context,meta),self.features)

    def _accept(self, prediction, source, current_index):
        validate_prediction(prediction,source,self.features)
        if prediction.model_revision != self.client.revision:
            raise ValueError("Prediction model revision differs")
        if source.context != self.context or self.episode != (source.basis.runtime_instance, source.basis.timeline_generation):
            return
        offset = source.execution_time_ns - self.origin
        if offset < 0 or offset % self.spec.period_ns:
            raise ValueError("Observation does not correspond to an action boundary")
        start = offset // self.spec.period_ns
        future = {start+i: ActionSample(tuple(a),source.basis,start+i,source.execution_time_ns+i*self.spec.period_ns)
                  for i,a in enumerate(prediction.actions) if start+i >= current_index}
        if not future:
            self.diagnostics.append("prediction_arrived_after_all_samples_elapsed")
            return
        self.buffer = future

    def prime(self, observation):
        self.reset()
        if self.pending:
            try:
                self.pending[0].result(timeout=10)
            except TimeoutError as exc:
                raise RuntimeError("Previous inference request is still outstanding") from exc
            except Exception:
                pass
            self.pending = None
        self.episode = (observation.instance,observation.episode)
        self.origin = observation.time_ns
        source = self._input(observation)
        start = time.monotonic_ns()
        prediction = self.client.predict(source)
        self.latencies_ns.append(time.monotonic_ns()-start)
        self._accept(prediction,source,0)
        self.last_request = 0
        if 0 not in self.buffer:
            raise ValueError("Initial prediction has no valid first action")

    def _poll(self, index):
        if self.pending and self.pending[0].done():
            future, source, started = self.pending
            self.pending = None
            try:
                prediction, elapsed = future.result()
                self.latencies_ns.append(elapsed)
                self._accept(prediction,source,index)
            except Exception as exc:
                self.diagnostics.append(f"prediction_rejected: {exc}")

    def _predict(self, source):
        start = time.monotonic_ns()
        prediction = self.client.predict(source)
        return prediction, time.monotonic_ns()-start

    def action(self, index, observation=None):
        self._poll(index)
        if observation and self.episode != (observation.instance,observation.episode):
            self.reset()
            return None
        if observation is not None and index-self.last_request >= 5 and self.pending is None:
            source = self._input(observation)
            self.pending = (self.pool.submit(self._predict,source),source,time.monotonic_ns())
            self.last_request = index
        self.buffer = {i:a for i,a in self.buffer.items() if i >= index}
        return self.buffer.pop(index,None)

    def close(self):
        self.reset()
        if hasattr(self.client,"close"):
            self.client.close()
        self.pool.shutdown(wait=True,cancel_futures=True)
