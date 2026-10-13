"""Inference identities are independent of ROS, recording and robot authority."""

from pydantic import BaseModel, ConfigDict, Field


class Contract(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", allow_inf_nan=False)


class Identity(Contract):
    runtime_instance: str = Field(min_length=1)
    timeline_generation: int = Field(ge=0)
    observation_sequence: int = Field(ge=0)
