from __future__ import annotations

from typing import Dict, List

from core.api import RobotAPI


def robot_to_urdf(delta: List[float]) -> List[float]:
    """Convert EE delta from robot frame to URDF world frame.

    TR4 Pro arms are side-mounted.  The observed axis correspondence is:
        robot +X (right)   = URDF +Z
        robot +Y (forward) = URDF -Y
        robot +Z (up)      = URDF +X

    Position transform:  [dx, dy, dz] -> [dz, -dy, dx]
    Rotation transform:  [roll, pitch, yaw] -> [yaw, -pitch, roll]
      (roll  about robot X = about URDF Z = URDF yaw)
      (pitch about robot Y = about -URDF Y = -URDF pitch)
      (yaw   about robot Z = about URDF X = URDF roll)
    """
    dx, dy, dz, roll, pitch, yaw = delta
    return [dz, -dy, dx, yaw, -pitch, roll]


class KeyboardInterface:
    """Keyboard -> EE delta command.

    Bindings use the robot frame (X=right, Y=forward, Z=up).
    robot_to_urdf() converts to URDF world frame before calling the API.
    """

    def __init__(self, translation_step: float = 0.01, rotation_step: float = 0.04) -> None:
        self.translation_step = float(translation_step)
        self.rotation_step = float(rotation_step)

        t = self.translation_step
        r = self.rotation_step

        # Deltas expressed in robot frame (X=right, Y=fwd, Z=up).
        self._bindings: Dict[str, List[float]] = {
            # Translation
            "KeyW": [0.0,  t,  0.0, 0.0, 0.0, 0.0],  # +Y forward
            "KeyS": [0.0, -t,  0.0, 0.0, 0.0, 0.0],  # -Y backward
            "KeyA": [-t,  0.0, 0.0, 0.0, 0.0, 0.0],  # -X left
            "KeyD": [ t,  0.0, 0.0, 0.0, 0.0, 0.0],  # +X right
            "KeyQ": [0.0, 0.0,  t,  0.0, 0.0, 0.0],  # +Z up
            "KeyE": [0.0, 0.0, -t,  0.0, 0.0, 0.0],  # -Z down
            # Rotation (robot frame)
            "KeyJ": [0.0, 0.0, 0.0,  r,  0.0, 0.0],  # roll+
            "KeyU": [0.0, 0.0, 0.0, -r,  0.0, 0.0],  # roll-
            "KeyK": [0.0, 0.0, 0.0, 0.0,  r,  0.0],  # pitch+
            "KeyI": [0.0, 0.0, 0.0, 0.0, -r,  0.0],  # pitch-
            "KeyL": [0.0, 0.0, 0.0, 0.0, 0.0,  r ],  # yaw+
            "KeyO": [0.0, 0.0, 0.0, 0.0, 0.0, -r ],  # yaw-
        }

    def delta_from_keys(self, keys: List[str]) -> List[float]:
        delta = [0.0] * 6
        for key in keys:
            values = self._bindings.get(key)
            if values is None:
                continue
            for idx in range(6):
                delta[idx] += values[idx]
        return delta

    def apply_keys(self, keys: List[str], api: RobotAPI) -> dict:
        delta_robot = self.delta_from_keys(keys)
        if all(abs(v) < 1e-12 for v in delta_robot):
            out = api.snapshot()
            out.update({"success": True, "moved": False, "keys": keys})
            return out

        delta_urdf = robot_to_urdf(delta_robot)
        out = api.step_ee(delta_urdf)
        out.update({"moved": True, "delta": delta_robot, "keys": keys})
        return out
