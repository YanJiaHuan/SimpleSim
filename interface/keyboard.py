from __future__ import annotations

from typing import Dict, List

from core.api import RobotAPI


class KeyboardInterface:
    """Keyboard -> EE delta command. No direct joint editing here."""

    def __init__(self, translation_step: float = 0.01, rotation_step: float = 0.05) -> None:
        self.translation_step = float(translation_step)
        self.rotation_step = float(rotation_step)

        t = self.translation_step
        r = self.rotation_step
        self._bindings: Dict[str, List[float]] = {
            "KeyW": [t, 0.0, 0.0, 0.0, 0.0, 0.0],
            "KeyS": [-t, 0.0, 0.0, 0.0, 0.0, 0.0],
            "KeyA": [0.0, t, 0.0, 0.0, 0.0, 0.0],
            "KeyD": [0.0, -t, 0.0, 0.0, 0.0, 0.0],
            "KeyR": [0.0, 0.0, t, 0.0, 0.0, 0.0],
            "KeyF": [0.0, 0.0, -t, 0.0, 0.0, 0.0],
            "KeyQ": [0.0, 0.0, 0.0, r, 0.0, 0.0],
            "KeyE": [0.0, 0.0, 0.0, -r, 0.0, 0.0],
            "ArrowUp": [0.0, 0.0, 0.0, 0.0, r, 0.0],
            "ArrowDown": [0.0, 0.0, 0.0, 0.0, -r, 0.0],
            "ArrowLeft": [0.0, 0.0, 0.0, 0.0, 0.0, r],
            "ArrowRight": [0.0, 0.0, 0.0, 0.0, 0.0, -r],
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

