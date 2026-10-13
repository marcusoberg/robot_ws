"""policy.v1 clients retain input identity and checkpoint identity."""

import asyncio
import time
import uuid
from websockets.sync.client import connect
from websockets.asyncio.client import connect as async_connect
from websockets.exceptions import WebSocketException
from .codec import pack, unpack, MAX_PACKET
from .contracts import Features, PolicyInput
from .decoder import PolicyDecoder


def _hello(reply, features, expected_revision):
    if (
        not isinstance(reply, dict)
        or reply.get("type") != "hello"
        or reply.get("protocol") != "policy.v1"
    ):
        raise ValueError(
            reply.get("reason", "Unexpected policy handshake")
            if isinstance(reply, dict)
            else "Invalid policy handshake"
        )
    if Features.model_validate(reply.get("features")) != features:
        raise ValueError("Policy feature negotiation changed within handshake")
    revision = reply.get("model_revision")
    if (
        not isinstance(revision, str)
        or not revision
        or (expected_revision is not None and revision != expected_revision)
    ):
        raise ValueError("Policy checkpoint differs from the selected artifact")
    return revision


def _prediction(reply, identifier, decoder, value, revision, start):
    if (
        not isinstance(reply, dict)
        or reply.get("type") != "prediction"
        or reply.get("id") != identifier
    ):
        raise ValueError(
            reply.get("reason", "Unexpected policy response")
            if isinstance(reply, dict)
            else "Invalid policy response"
        )
    prediction = decoder.prediction(reply.get("prediction"), value)
    if prediction.model_revision != revision:
        raise ValueError("Policy checkpoint changed within this connection")
    return prediction, {
        "round_trip_ns": time.monotonic_ns() - start,
        "compute_ns": prediction.computation_ns,
        "one_way_latency_ns": None,
    }


class RemoteModel:
    def __init__(self, url, features: Features, *, expected_revision=None, timeout=120):
        self.features, self.timeout = features, timeout
        self.decoder = PolicyDecoder(features)
        try:
            self.ws = connect(
                url, max_size=MAX_PACKET, compression=None, open_timeout=timeout
            )
            self.ws.send(
                pack({"protocol": "policy.v1", "features": features.model_dump()})
            )
            self.revision = _hello(
                unpack(self.ws.recv(timeout=timeout)), features, expected_revision
            )
        except Exception:
            if hasattr(self, "ws"):
                self.ws.close()
            raise

    def predict(self, value: PolicyInput):
        value = self.decoder.input(value.model_dump())
        identifier, start = uuid.uuid4().hex, time.monotonic_ns()
        try:
            self.ws.send(pack({"id": identifier, "input": value.model_dump()}))
            reply = unpack(self.ws.recv(timeout=self.timeout))
        except (OSError, WebSocketException, TimeoutError) as error:
            raise RuntimeError(
                f"Policy inference connection failed: {error}"
            ) from error
        return _prediction(reply, identifier, self.decoder, value, self.revision, start)

    def close(self):
        self.ws.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class AsyncRemoteModel:
    @classmethod
    async def connect(
        cls, url, features: Features, *, expected_revision=None, timeout=120
    ):
        self = cls()
        self.features, self.timeout = features, timeout
        self.decoder, self.lock = PolicyDecoder(features), asyncio.Lock()
        self.ws = await async_connect(
            url, max_size=MAX_PACKET, compression=None, open_timeout=timeout
        )
        try:
            await self.ws.send(
                pack({"protocol": "policy.v1", "features": features.model_dump()})
            )
            self.revision = _hello(
                unpack(await asyncio.wait_for(self.ws.recv(), timeout)),
                features,
                expected_revision,
            )
        except Exception:
            await self.ws.close()
            raise
        return self

    async def predict(self, value: PolicyInput):
        value = self.decoder.input(value.model_dump())
        async with self.lock:
            identifier, start = uuid.uuid4().hex, time.monotonic_ns()
            try:
                await self.ws.send(
                    pack({"id": identifier, "input": value.model_dump()})
                )
                reply = unpack(await asyncio.wait_for(self.ws.recv(), self.timeout))
            except (OSError, WebSocketException, TimeoutError) as error:
                raise RuntimeError(
                    f"Policy inference connection failed: {error}"
                ) from error
            return _prediction(
                reply, identifier, self.decoder, value, self.revision, start
            )

    async def close(self):
        await self.ws.close()

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        await self.close()
