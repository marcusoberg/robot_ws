"""Synchronous inference transport, used outside the ROS callback thread."""
import time
import uuid
from websockets.sync.client import connect
from . import protocol_pb2 as pb
from .codec import *
from .contracts import validate_input, validate_prediction

class RemoteModel:
    def __init__(self, url, features, *, expected_revision=None, timeout=120):
        self.features, self.timeout = features, timeout
        self.ws = connect(url, max_size=MAX_PACKET, max_queue=1, open_timeout=timeout)
        try:
            self.ws.send(envelope("hello", pb.Hello(features=features_to_pb(features),
                expected_revision=expected_revision or "")))
            reply = decode(self.ws.recv(timeout=timeout))
            if reply.WhichOneof("payload") == "error":
                raise ValueError(reply.error.reason)
            if reply.WhichOneof("payload") != "ready" or features_from_pb(reply.ready.features) != features:
                raise ValueError("Inference feature handshake mismatch")
            self.revision = reply.ready.model_revision
            if not self.revision or expected_revision and self.revision != expected_revision:
                raise ValueError("Unexpected model deployment identity")
        except Exception:
            self.ws.close()
            raise

    def predict(self, value):
        validate_input(value, self.features)
        request_id, start = uuid.uuid4().hex, time.monotonic_ns()
        self.ws.send(envelope("input", input_to_pb(value), request_id))
        reply = decode(self.ws.recv(timeout=self.timeout))
        if reply.request_id != request_id:
            raise ValueError("Uncorrelated inference response")
        if reply.WhichOneof("payload") == "error":
            raise RuntimeError(reply.error.reason)
        if reply.WhichOneof("payload") != "prediction":
            raise ValueError("Expected prediction")
        prediction = validate_prediction(prediction_from_pb(reply.prediction), value, self.features)
        if prediction.model_revision != self.revision:
            raise ValueError("Model deployment changed during connection")
        return prediction, {"round_trip_ns": time.monotonic_ns()-start,
                            "compute_ns": prediction.computation_ns, "one_way_latency_ns": None}

    def close(self):
        self.ws.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
