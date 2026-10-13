"""Snapshot-derived scene transforms; this workspace never advances physics."""

import numpy as np


class SceneFrames:
    def __init__(self, configuration, description):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(configuration.mujoco.model))
        self.data = mujoco.MjData(self.model)
        name = configuration.mujoco.base_frame or description["base"]
        for kind, rotation, position in (
            (mujoco.mjtObj.mjOBJ_BODY, "xmat", "xpos"),
            (mujoco.mjtObj.mjOBJ_SITE, "site_xmat", "site_xpos"),
        ):
            identifier = mujoco.mj_name2id(self.model, kind, name)
            if identifier >= 0:
                self.identifier, self.rotation, self.position = (
                    identifier,
                    rotation,
                    position,
                )
                break
        else:
            raise ValueError(f"Scene has no configured robot base frame {name!r}")

    def base_from_world(self, observation):
        self.mj.mj_setState(
            self.model,
            self.data,
            observation.sensors["mujoco/integration_state"].values,
            self.mj.mjtState.mjSTATE_INTEGRATION,
        )
        self.mj.mj_forward(self.model, self.data)
        rotation = getattr(self.data, self.rotation)[self.identifier].reshape(3, 3)
        position = getattr(self.data, self.position)[self.identifier]
        inverse = np.eye(4)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -rotation.T @ position
        return inverse
