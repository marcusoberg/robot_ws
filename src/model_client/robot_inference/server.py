"""One model, one active client, binary Protobuf over WebSockets."""
import argparse
import asyncio
import os
from pathlib import Path
from websockets.asyncio.server import serve
from . import protocol_pb2 as pb
from .codec import (MAX_PACKET, decode, envelope, features_from_pb, features_to_pb,
                    input_from_pb, prediction_to_pb)
from .contracts import Features, validate_input, validate_prediction
from .bundle import verify
from .lerobot import LeRobotModel

async def run(model, host, port):
    occupied = False
    async def connection(ws):
        nonlocal occupied
        if occupied:
            await ws.send(envelope("error", pb.Error(reason="A model client is already connected")))
            return
        occupied = True
        request_id = ""
        try:
            hello = decode(await asyncio.wait_for(ws.recv(), 30))
            if hello.WhichOneof("payload") != "hello" or features_from_pb(hello.hello.features) != model.describe():
                raise ValueError("Inference feature contract differs from deployed model")
            if hello.hello.expected_revision and hello.hello.expected_revision != model.revision:
                raise ValueError("Requested deployment identity differs from served model")
            # A reconnect is a new inference context even if the client reused its label.
            model.context = None
            await ws.send(envelope("ready", pb.Ready(
                features=features_to_pb(model.describe()), model_revision=model.revision)))
            async for message in ws:
                packet = decode(message)
                request_id = packet.request_id
                if packet.WhichOneof("payload") != "input" or not request_id:
                    raise ValueError("Expected an identified inference input")
                value = validate_input(input_from_pb(packet.input), model.describe())
                prediction = await asyncio.to_thread(model.predict, value)
                validate_prediction(prediction, value, model.describe())
                await ws.send(envelope("prediction", prediction_to_pb(prediction), request_id))
        except Exception as error:
            try:
                await ws.send(envelope("error", pb.Error(reason=str(error)), request_id))
            except Exception:
                pass
        finally:
            occupied = False
    async with serve(connection, host, port, max_size=MAX_PACKET, max_queue=1, compression=None):
        print(f"Model ready: ws://{host}:{port} (robot.inference.v2) revision={model.revision}", flush=True)
        await asyncio.Future()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bundle", type=Path, required=True)
    p.add_argument("--device", choices=("cpu", "mps", "cuda"), required=True)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--offline", action="store_true")
    a = p.parse_args()
    manifest = verify(a.bundle)
    if a.offline:
        os.environ.update(HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_DATASETS_OFFLINE="1")
    cache = a.bundle / "huggingface"
    if cache.exists():
        os.environ["HF_HUB_CACHE"] = str(cache.resolve())
    features = Features.from_json((a.bundle / "features.json").read_text())
    model = LeRobotModel.load(str(a.bundle / "model"), features=features,
        revision=manifest["revision"], device=a.device, model_type=manifest["model_type"])
    model.revision = manifest["revision"]
    try:
        asyncio.run(run(model, a.host, a.port))
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
