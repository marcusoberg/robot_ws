"""Inference transport and serving; no ROS or robot SDK dependency."""
from .contracts import Features, Identity, PolicyInput, Prediction
from .client import RemoteModel
