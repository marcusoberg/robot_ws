"""Forward kinematics for an ordered set of bounded scalar URDF joints."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin


class UrdfModel:
    def __init__(self, description, joints, base, tip):
        self.description = Path(description)
        self.joints = tuple(joints)
        defined = {
            joint.attrib["name"]: joint
            for joint in ET.parse(self.description).getroot().findall("joint")
            if joint.attrib["type"] != "fixed" and joint.find("mimic") is None
        }
        if len(set(self.joints)) != len(self.joints) or set(defined) != set(self.joints):
            raise ValueError("Configured joints must name every movable non-mimic URDF joint exactly once")

        lower, upper, units = [], [], []
        for name in self.joints:
            joint = defined[name]
            limit = joint.find("limit")
            if (joint.attrib["type"] not in ("revolute", "prismatic") or limit is None
                    or not {"lower", "upper"} <= limit.attrib.keys()):
                raise ValueError(f"A bounded revolute or prismatic joint is required: {name}")
            lower.append(float(limit.attrib["lower"]))
            upper.append(float(limit.attrib["upper"]))
            units.append("rad" if joint.attrib["type"] == "revolute" else "m")
        self.lower = tuple(lower)
        self.upper = tuple(upper)
        self.units = tuple(units)
        if not np.isfinite(lower + upper).all() or np.any(np.asarray(lower) >= upper):
            raise ValueError("Joint limits must be finite and strictly ordered")

        self.model = pin.buildModelFromUrdf(str(self.description), mimic=True)
        self.data = self.model.createData()
        model_joints = [self.model.joints[self.model.getJointId(name)] for name in self.joints]
        if any(joint.nq != 1 or joint.nv != 1 for joint in model_joints):
            raise ValueError("Kinematics requires scalar joints")
        self.indices = [joint.idx_q for joint in model_joints]
        self.velocity_indices = np.array([joint.idx_v for joint in model_joints], dtype=int)
        for frame in (base, tip):
            if not self.model.existFrame(frame):
                raise ValueError(f"Unknown URDF frame: {frame}")
        self.base = self.model.getFrameId(base)
        self.tip = self.model.getFrameId(tip)
        if self.model.frames[self.base].parentJoint != 0:
            raise ValueError("The configured base must be fixed to the URDF root")
        chain = set()
        joint_id = self.model.frames[self.tip].parentJoint
        while joint_id:
            chain.add(self.model.names[joint_id])
            joint_id = self.model.parents[joint_id]
        self.active = np.array([i for i, name in enumerate(self.joints) if name in chain], dtype=int)
        if not len(self.active):
            raise ValueError("No movable joints between the configured base and tip")

    def positions(self, positions):
        """Copy a finite joint vector; measured seeds may lie outside limits."""
        values = np.asarray(positions, dtype=float)
        if values.shape != (len(self.joints),) or not np.isfinite(values).all():
            raise ValueError("Joint state requires one finite position per configured joint")
        return values.copy()

    def validate(self, positions):
        """Validate a command against the URDF position bounds."""
        values = self.positions(positions)
        outside = np.flatnonzero((values < self.lower) | (values > self.upper))
        if len(outside):
            details = "; ".join(
                f"{self.joints[i]}={values[i]:.9g} {self.units[i]} outside "
                f"[{self.lower[i]:.9g}, {self.upper[i]:.9g}]" for i in outside
            )
            raise ValueError(f"Joint positions outside the declared bounds: {details}")
        return values

    def pose(self, positions):
        """Evaluate FK; inputs in the solver are already validated joint vectors."""
        q = pin.neutral(self.model)
        q[self.indices] = positions
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.base].inverse() * self.data.oMf[self.tip]

    def jacobian(self, positions, reference_frame):
        q = pin.neutral(self.model)
        q[self.indices] = positions
        jacobian = pin.computeFrameJacobian(self.model, self.data, q, self.tip, reference_frame)
        return jacobian[:, self.velocity_indices[self.active]]

    def integrate(self, positions, increment):
        q = pin.neutral(self.model)
        q[self.indices] = positions
        velocity = np.zeros(self.model.nv)
        velocity[self.velocity_indices[self.active]] = increment
        return pin.integrate(self.model, q, velocity)[self.indices]
