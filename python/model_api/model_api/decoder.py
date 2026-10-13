"""One network boundary for the feature contract negotiated before control."""

import numpy as np
from .contracts import Features, PolicyInput, Prediction


class PolicyDecoder:
    def __init__(self, features: Features):
        self.features = features
        self.shapes = {
            "observation.state": (len(features.state_joints),),
            **{
                name: (shape[1], shape[2], shape[0])
                for name, shape in features.cameras.items()
            },
        }

    def input(self, packet) -> PolicyInput:
        value = PolicyInput.model_validate(packet)
        if value.features.keys() != self.shapes.keys():
            raise ValueError("Input feature names differ from the negotiated profile")
        for name, shape in self.shapes.items():
            array = np.asarray(value.features[name])
            if array.shape != shape:
                raise ValueError(f"{name} expects {shape}, received {array.shape}")
            if name in self.features.cameras:
                if array.dtype != np.uint8:
                    raise ValueError(f"{name} requires RGB uint8 pixels")
            elif not np.isfinite(array).all():
                raise ValueError(f"{name} contains non-finite coordinates")
        if self.features.instruction and not value.instruction.strip():
            raise ValueError("This model requires a task instruction")
        return value

    def prediction(self, packet, source: PolicyInput) -> Prediction:
        value = Prediction.model_validate(packet)
        actions = np.asarray(value.actions)
        if (
            value.basis != source.basis
            or value.execution_time_ns != source.execution_time_ns
            or value.action_period != self.features.action_period
            or actions.ndim != 2
            or actions.shape[1] != len(self.features.state_joints)
            or actions.shape[0] == 0
            or not np.isfinite(actions).all()
        ):
            raise ValueError(
                "Prediction disagrees with its input basis or negotiated action contract"
            )
        return value
