"""Canonical MCAP JSON envelopes. Pixel packets remain lossless and transport-neutral."""

import base64
from pydantic import Field
from recording_tools.contracts.records import Contract, CaptureRecord
from recording_tools.contracts.codec import pack, unpack


class RecordedCapture(Contract):
    record: CaptureRecord
    image_packet: str = Field(
        json_schema_extra={
            "contentEncoding": "base64",
            "contentMediaType": "application/msgpack",
        }
    )


def encode_capture(record: CaptureRecord, images: dict) -> bytes:
    return (
        RecordedCapture(
            record=record, image_packet=base64.b64encode(pack(images)).decode("ascii")
        )
        .model_dump_json()
        .encode()
    )


def decode_capture(encoding: str, payload: bytes):
    match encoding:
        case "json":
            value = RecordedCapture.model_validate_json(payload)
            return value.record, unpack(
                base64.b64decode(value.image_packet, validate=True)
            )
        case "messagepack":
            # Read the initial migration recordings without mutating their artifacts.
            value = unpack(payload)
            return CaptureRecord.model_validate(value["record"]), value["images"]
        case _:
            raise ValueError(f"Unsupported canonical capture encoding: {encoding}")


def decode_capture_metadata(encoding: str, payload: bytes) -> CaptureRecord:
    """Index capture identities without retaining decoded image planes."""
    match encoding:
        case "json":
            return RecordedCapture.model_validate_json(payload).record
        case "messagepack":
            return CaptureRecord.model_validate(unpack(payload)["record"])
        case _:
            raise ValueError(f"Unsupported canonical capture encoding: {encoding}")
