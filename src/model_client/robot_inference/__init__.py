"""Inference transport and serving; no ROS or robot SDK dependency."""
from robot_core.data.inference import Features, Identity, PolicyInput, Prediction
from .client import RemoteModel
