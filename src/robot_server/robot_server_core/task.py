import itertools
import numpy as np

class PickPlaceSuccess:
    def __init__(self):
        self.since = None
        self.success = False

    def update(self, snapshot):
        if "block" not in snapshot.bodies or "container" not in snapshot.bodies:
            raise ValueError("Fixed-scene evaluation requires cube and container geometry")
        cube = np.array(snapshot.bodies["block"][:3])
        rotation = np.array(snapshot.bodies["block"][3:]).reshape(3,3)
        base = np.array(snapshot.bodies["container"][:3])
        axes = np.array(snapshot.bodies["container"][3:]).reshape(3,3)
        supported, gripped = False, False
        for contact in snapshot.contacts:
            if contact.normal_force <= 1e-4:
                continue
            if contact.geom1 == "block":
                other, ancestors = contact.geom2, contact.ancestors2
            elif contact.geom2 == "block":
                other, ancestors = contact.geom1, contact.ancestors1
            else:
                continue
            supported |= other == "container_bottom"
            gripped |= "gripper" in ancestors or "base" in ancestors
        corners = np.array(list(itertools.product((-.015,.015), repeat=3))) @ rotation.T + cube
        local = (corners-base) @ axes
        valid = (np.all(np.abs(local[:,:2]) <= .04) and supported and not gripped)
        if not valid:
            self.since = None
        elif self.since is None:
            self.since = snapshot.time
        if self.since is not None and snapshot.time-self.since >= 1_000_000_000:
            self.success = True
        return self.success
