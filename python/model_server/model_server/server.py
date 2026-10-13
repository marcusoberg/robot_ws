"""policy.v1: model workers have neither ROS discovery nor robot authority."""

import asyncio
import argparse
import json
import os
from http import HTTPStatus
from pathlib import Path
from websockets.asyncio.server import serve

from model_api.codec import pack, unpack, MAX_PACKET
from model_api.decoder import PolicyDecoder
from model_api.contracts import Features, PolicyInput, Prediction

from .lerobot import LeRobotModel


async def run(model, host, port):
    # Warm-up happens on the client handshake before authority is acquired.
    lock = asyncio.Lock()
    decoder = PolicyDecoder(model.describe())
    model_features: Features = model.describe()

    async def connection(ws):
        try:
            hello = unpack(await asyncio.wait_for(ws.recv(), 30))
            if (
                not isinstance(hello, dict)
                or hello.get("protocol") != "policy.v1"
                or Features.model_validate(hello.get("features")) != model_features
            ):
                raise ValueError("Incompatible policy feature contract")
        except (ValueError, TypeError, TimeoutError, KeyError) as error:
            await ws.send(pack({"type": "error", "reason": str(error)}))
            return

        await ws.send(
            pack({
                "type": "hello",
                "protocol": "policy.v1",
                "model_revision": model.revision,
                "features": model_features.model_dump(),
            })
        )

        async for message in ws:
            try:
                packet = unpack(message)
                if (
                    not isinstance(packet, dict)
                    or set(packet) != {"id", "input"}
                    or not isinstance(packet["id"], str)
                    or not 0 < len(packet["id"]) <= 128
                ):
                    raise ValueError("Expected a bounded request id and policy input")

                value = decoder.input(packet["input"])
                async with lock:
                    prediction = await asyncio.to_thread(model.predict, value)

                await ws.send(
                    pack(
                        {
                            "type": "prediction",
                            "id": packet["id"],
                            "prediction": prediction.model_dump(),
                        }
                    )
                )
            except Exception as error:
                await ws.send(pack({"type": "error", "reason": str(error)}))

    def health(conn, req):
        if req.path == "/health":
            return conn.respond(
                HTTPStatus.OK,
                json.dumps({"protocol": "policy.v1", "model_revision": model.revision}),
            )
        return None

    async with serve(
        connection,
        host,
        port,
        max_size=MAX_PACKET,
        max_queue=1,
        compression=None,
        process_request=health,
    ):
        print(f"Policy ready: ws://{host}:{port} (policy.v1)", flush=True)
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--features", required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--model", choices=["act", "smolvla"], required=True)
    parser.add_argument("--device", choices=["cpu", "mps", "cuda"], required=True)
    parser.add_argument(
        "--offline",
        action="store_true",
        help="Require cached tokenizer/config resources",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    if not Path(args.checkpoint).is_dir():
        parser.error("--checkpoint must name a local checkpoint directory")

    if args.offline:
        os.environ.update(HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_DATASETS_OFFLINE="1")
    features = Features.model_validate_json(Path(args.features).read_text())
    try:
        asyncio.run(
            run(
                LeRobotModel.load(
                    args.checkpoint,
                    features=features,
                    revision=args.revision,
                    device=args.device,
                    model_type=args.model,
                ),
                args.host,
                args.port,
            )
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
