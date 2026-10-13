"""ROS-free robot SDK. Native execution owns timing, IK and interpolation."""
from .configuration import Configuration, RobotConfiguration, ControllerConfiguration, MujocoConfiguration, SerialConfiguration, ControlLimit, SensorConfiguration, HostTiming, Camera
from .types import Observation, ObservationIdentity, Capture, ImageSample, SampleStamp, JointTarget, JointVelocity, Pose, Twist, Waypoint, Trajectory, ReferencePoint, ReferenceChunk, TimedJointTargets, Mode, RobotError, AuthorityLost, OperationRejected, MotionRejected, EventHistoryLost, StaleObservation, CameraError
from .robot import Robot, ControlSource, ControlSession, Operation, MotionOperation, OperationResult, Outcome


def fit_timed_targets(actions, origin_ns, period_ns, lower, upper, execution_limits, tolerance):
    """Fit absolute target knots with the native algorithm, without opening a robot."""
    import numpy as np
    from robot_runtime_native import _native
    budget = [float(tolerance)] if np.isscalar(tolerance) else list(tolerance)
    return _native.fit_timed_targets(actions, origin_ns, period_ns, lower, upper,
                                    execution_limits["velocity"], execution_limits["acceleration"], execution_limits["jerk"], budget)
