"""One bounded writer lane. Live overflow is explicit; stepped callers await tickets."""

from concurrent.futures import Future
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from queue import Queue, Full
from threading import Thread, Lock
import json
import time
from mcap.writer import Writer, CompressionType
from recording_tools.contracts import CaptureRecord, Event
from .encoding import RecordedCapture, encode_capture


@dataclass(frozen=True)
class Write:
    topic: str
    payload: bytes
    log_ns: int
    publish_ns: int
    encoding: str
    schema: bytes
    future: Future
    schema_name: str
    schema_encoding: str


class Recorder:
    def __init__(self, directory: str | Path, manifest: dict, capacity: int = 64):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.manifest: dict[str, Any] = {
            **manifest,
            "format": "robot.recording.v1",
            "complete": False,
            "dropped": 0,
        }
        self.queue = Queue(capacity)
        self.lock = Lock()
        self.error = None
        self.closed = False
        self.thread = Thread(target=self._write, name="mcap-writer", daemon=False)
        self.thread.start()

    def append(
        self,
        topic,
        payload,
        *,
        publish_ns=None,
        encoding="json",
        schema=b"",
        schema_name=None,
        schema_encoding="jsonschema",
        wait=False,
    ) -> Future:
        future = Future()
        now = time.time_ns()
        if self.closed:
            raise RuntimeError("Recording is closed")
        if self.error:
            raise RuntimeError("MCAP writer failed") from self.error
        item = Write(
            topic,
            payload,
            now,
            publish_ns or now,
            encoding,
            schema,
            future,
            schema_name or topic,
            schema_encoding,
        )
        try:
            self.queue.put(item, block=wait, timeout=10 if wait else None)
        except Full:
            with self.lock:
                self.manifest["dropped"] = int(self.manifest["dropped"]) + 1
            future.set_exception(
                BufferError("Recording queue overflow; this run is incomplete")
            )
        return future

    def capture(self, record: CaptureRecord, images: dict, *, wait=False) -> Future:
        # One message atomically retains all planes and the original state basis.
        return self.append(
            "/robot/capture",
            encode_capture(record, images),
            encoding="json",
            schema=json.dumps(RecordedCapture.model_json_schema()).encode(),
            schema_name="robot.recording.capture.v1",
            wait=wait,
        )

    def event(self, event: Event, *, wait=False) -> Future:
        return self.append(
            "/experiment/events",
            event.model_dump_json().encode(),
            schema=json.dumps(Event.model_json_schema()).encode(),
            wait=wait,
        )

    def _write(self):
        channels = {}
        sequences = {}
        try:
            with (self.directory / "recording.mcap").open("wb") as stream:
                writer = Writer(stream, compression=CompressionType.ZSTD)
                writer.start(profile="robot.data.v1")
                while (item := self.queue.get()) is not None:
                    try:
                        key = (item.topic, item.encoding, item.schema)
                        if key not in channels:
                            schema_id = (
                                writer.register_schema(
                                    item.schema_name, item.schema_encoding, item.schema
                                )
                                if item.schema
                                else 0
                            )
                            channels[key] = writer.register_channel(
                                item.topic, item.encoding, schema_id
                            )
                        sequences[key] = sequences.get(key, 0) + 1
                        writer.add_message(
                            channels[key],
                            item.log_ns,
                            item.payload,
                            item.publish_ns,
                            sequences[key],
                        )
                        stream.flush()
                        item.future.set_result(sequences[key])
                    except BaseException as error:
                        item.future.set_exception(error)
                        raise
                writer.add_metadata(
                    "robot.run", {"manifest": json.dumps(self.manifest)}
                )
                writer.finish()
        except BaseException as error:
            self.error = error
            while not self.queue.empty():
                item = self.queue.get_nowait()
                if item is not None:
                    item.future.set_exception(error)

    def close(self):
        if self.closed:
            return
        self.closed = True
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=0.2)
                break
            except Full:
                continue
        self.thread.join(timeout=30)
        self.manifest["complete"] = (
            not self.thread.is_alive()
            and self.error is None
            and self.manifest["dropped"] == 0
            and self.manifest.get("capture_gaps", 0) == 0
        )
        self.manifest["error"] = str(self.error) if self.error else None
        (self.directory / "manifest.json").write_text(
            json.dumps(self.manifest, indent=2)
        )
        if not self.manifest["complete"]:
            raise RuntimeError("Incomplete recording; inspect manifest.json")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
