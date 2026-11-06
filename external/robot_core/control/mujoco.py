"""Execute timed joint moves with MuJoCo's position actuators in simulation time."""

from concurrent.futures import Future, ThreadPoolExecutor
from itertools import count
import math

import mujoco
import numpy as np

from robot_core.base.command import CommandAction
from robot_core.config import robot_settings
from robot_core.validation import joint_positions, timed_move
from robot_core.control.joint_move import JointMoveProfile
from robot_core.data.commands import MoveJoint
from robot_core.data.spec import Profile


class MujocoController(CommandAction[MoveJoint]):
    def __init__(self, profile: Profile):
        self.profile = profile
        self.model = mujoco.MjModel.from_xml_path(str(profile.scene))
        self.data = mujoco.MjData(self.model)
        joints = [self.model.joint(name).id for name in profile.spec.joints]
        self.qpos = self.model.jnt_qposadr[joints]
        self.arm_dofs = self.model.jnt_dofadr[joints[:6]]
        # Map each named joint to its actuator independently of MJCF ordering.
        drive = self.model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT
        self.actuators = []
        for name, joint in zip(profile.spec.joints, joints):
            matches = np.flatnonzero(drive & (self.model.actuator_trnid[:, 0] == joint))
            actuator, = matches
            self.actuators.append(int(actuator))
        self.arm_actuators = self.actuators[:6]
        self.gear = self.model.actuator_gear[self.arm_actuators, 0]
        self.kp = self.model.actuator_gainprm[self.arm_actuators, 0]
        initial = np.array(profile.initial, dtype=float)
        self.endpoint_tolerance = robot_settings()['motion']['endpoint_tolerance_rad']
        self.reference = initial
        self.data.qpos[self.qpos] = initial
        self.data.ctrl[self.actuators] = initial
        mujoco.mj_forward(self.model, self.data)
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mujoco")
        self.indices = count()

    def execute(self, command: MoveJoint) -> Future[int]:
        timed_move(command)
        joint_positions(self.profile.spec, [*command.angles, self.profile.initial[-1]])
        return self.worker.submit(self._move, command)

    def _move(self, command):
        # All writes to this MjData take place on the same worker. Start each
        # profile at the preceding command's endpoint and hold its gripper target.
        move = JointMoveProfile(self.reference, command)
        dt = self.model.opt.timestep
        for step in range(1, math.ceil(move.duration / dt) + 1):
            self._step(move.at(step * dt))
        settle_steps = math.ceil(self.profile.stop_timeout_ns / 1e9 / dt)
        for step in range(settle_steps + 1):
            errors = np.abs(self.data.qpos[self.qpos[:6]] - move.target[:6])
            if np.max(errors) <= self.endpoint_tolerance:
                return next(self.indices)
            if step < settle_steps:
                self._step(move.target)
        joint = int(np.argmax(errors))
        raise TimeoutError(f"{self.profile.spec.joints[joint]} is {errors[joint]:.5f} rad from the MuJoCo target")

    def _step(self, reference):
        self.reference = reference
        self.data.ctrl[self.actuators] = reference
        # Apply the model's gravity/Coriolis feed-forward through the actual
        # position servos, so their configured torque limits still apply.
        self.data.ctrl[self.arm_actuators] = (
            reference[:6] * self.gear + self.data.qfrc_bias[self.arm_dofs] / (self.gear * self.kp)
        )
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def close(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
