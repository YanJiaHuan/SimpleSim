from __future__ import annotations

from typing import Dict, List

from core.api import RobotAPI


class KeyboardInterface:
    """Keyboard -> EE delta command. No direct joint editing here.

    Coordinate convention (x+ right, y+ forward, z+ up):
      W/S  — translate Y (forward / backward)
      A/D  — translate X (left / right)
      Q/E  — translate Z (up / down)
      J/U  — rotate roll  (+/-)
      K/I  — rotate pitch (+/-)
      L/O  — rotate yaw   (+/-)
    """

    def __init__(self, translation_step: float = 0.01, rotation_step: float = 0.04) -> None:
        self.translation_step = float(translation_step)
        self.rotation_step = float(rotation_step)

        t = self.translation_step
        r = self.rotation_step
        self._bindings: Dict[str, List[float]] = {
            # Translation
            "KeyW": [0.0, -t,  0.0, 0.0, 0.0, 0.0],  # -Y forward (URDF Y+ = robot back)
            "KeyS": [0.0,  t,  0.0, 0.0, 0.0, 0.0],  # +Y backward
            "KeyD": [ t,  0.0, 0.0, 0.0, 0.0, 0.0],  # +X right
            "KeyA": [-t,  0.0, 0.0, 0.0, 0.0, 0.0],  # -X left
            "KeyQ": [0.0, 0.0,  t,  0.0, 0.0, 0.0],  # +Z up
            "KeyE": [0.0, 0.0, -t,  0.0, 0.0, 0.0],  # -Z down
            # Rotation
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
        delta = self.delta_from_keys(keys)
        if all(abs(value) < 1e-12 for value in delta):
            out = api.snapshot()
            out.update({"success": True, "moved": False, "keys": keys})
            return out

        out = api.step_ee(delta)
        out.update({"moved": True, "delta": delta, "keys": keys})
        return out
