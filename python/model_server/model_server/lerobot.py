"""Optional LeRobot model adapter. Importing model_server does not import Torch."""

from pathlib import Path
import time
import hashlib
import numpy as np
from model_api import Features, PolicyInput, Prediction


class LeRobotModel:
    @classmethod
    def load(
        cls,
        checkpoint: str,
        *,
        features: Features,
        revision: str,
        device: str,
        model_type: str,
    ):
        return cls(
            checkpoint,
            features,
            revision=revision,
            device=device,
            model_type=model_type,
        )

    def __init__(
        self,
        checkpoint: str,
        features: Features,
        *,
        revision: str,
        device="cpu",
        model_type="act",
    ):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        if device not in {"cpu", "mps", "cuda"}:
            raise ValueError("Choose an explicit model device: cpu, mps or cuda")
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")
        if device == "mps" and not torch.backends.mps.is_available():
            raise RuntimeError("MPS was requested but is unavailable")

        self.device = device
        torch.set_num_threads(4)
        self.torch = torch
        self.features = features
        self.source_revision = revision
        with (Path(checkpoint) / "model.safetensors").open("rb") as weights:
            self.revision = ("sha256:" + hashlib.file_digest(weights, "sha256").hexdigest())
        self.context = None

        get_policy_class(model_type)  # Registers the selected configuration before decoding it.
        config = PreTrainedConfig.from_pretrained(checkpoint)
        config.device = device

        if config.type != model_type:
            raise ValueError(f"Checkpoint type {config.type} differs from profile {model_type}")
        if model_type == "act":
            config.pretrained_backbone_weights = None
        elif model_type == "smolvla":
            # A full policy checkpoint contains the VLM parameters. Construct its
            # architecture without downloading a second set of weights to overwrite.
            config.load_vlm_weights = False

        configured = {
            "observation.state": [len(features.state_joints)],
            **{k: list(v) for k, v in features.cameras.items()},
        }
        actual = {key: list(v.shape) for key, v in config.input_features.items()}

        if actual != configured or list(config.output_features["action"].shape) != [len(features.state_joints)]:
            raise ValueError(
                f"Checkpoint feature contract differs: {actual}. Fine-tune with this profile's dataset first."
            )

        self.policy = (
            get_policy_class(config.type)
            .from_pretrained(checkpoint, config=config, strict=True)
            .to(device)
            .eval()
        )
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=config,
            pretrained_path=checkpoint,
            preprocessor_overrides={"device_processor": {"device": device}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )

    def describe(self):
        return self.features

    def reset(self, episode, context):
        if (episode, context) != self.context:
            self.policy.reset()
            self.context = (episode, context)

    def predict(self, value: PolicyInput) -> Prediction:
        self.reset(value.episode, value.context)
        torch = self.torch
        batch = {
            "observation.state": torch.as_tensor(
                np.asarray(value.features["observation.state"]).copy(),
                dtype=torch.float32,
            ).unsqueeze(0),
            "task": [value.instruction],
        }

        for feature in self.features.cameras:
            pixels = np.asarray(value.features[feature])
            batch[feature] = (
                torch.from_numpy(pixels.copy())
                .permute(2, 0, 1)
                .float()
                .div_(255)
                .unsqueeze(0)
            )

        start = time.monotonic_ns()
        with torch.inference_mode():
            actions = self.policy.predict_action_chunk(self.pre(batch))
            decoded = (
                torch.stack(
                    [self.post(actions[:, i])[0] for i in range(actions.shape[1])]
                )
                .cpu()
                .numpy()
            )

        if not np.isfinite(decoded).all():
            raise ValueError("Model returned nonfinite actions")
        return Prediction(
            basis=value.basis,
            execution_time_ns=value.execution_time_ns,
            model_revision=self.revision,
            action_representation="joint_position",
            action_period=self.features.action_period,
            actions=decoded.tolist(),
            computation_ns=time.monotonic_ns() - start,
        )

