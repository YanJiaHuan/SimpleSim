from __future__ import annotations

import threading
from typing import Any, Dict, List


class AccessoryController:
    """Manages robot accessories (linear axis, grippers) outside IK chains."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self._lock = threading.RLock()

        self._accessories: Dict[str, Dict[str, Any]] = {}
        for item in config.get("accessories", []):
            accessory_id = str(item["id"])
            kind = str(item["kind"])
            base: Dict[str, Any] = {
                "kind": kind,
                "label": str(item.get("label", accessory_id)),
                "arm": item.get("arm"),
            }
            if kind == "linear_axis":
                lower = float(item.get("lower", 0.0))
                upper = float(item.get("upper", 0.0))
                init = float(item.get("init", lower))
                base.update(
                    {
                        "joint_name": str(item["joint_name"]),
                        "lower": lower,
                        "upper": upper,
                        "step": float(item.get("step", 0.01)),
                        "value": max(lower, min(upper, init)),
                    }
                )
            elif kind == "gripper":
                joint_names = [str(n) for n in item.get("joint_names", [])]
                n = len(joint_names)
                base.update(
                    {
                        "joint_names": joint_names,
                        "open_positions": [float(v) for v in item.get("open_positions", [0.0] * n)],
                        "closed_positions": [float(v) for v in item.get("closed_positions", [0.0] * n)],
                        "aperture": 0.0,
                    }
                )
            else:
                continue
            self._accessories[accessory_id] = base

    def apply_keys(self, keys: List[str], active_arm: str) -> None:
        with self._lock:
            for item in self._accessories.values():
                if item["kind"] == "linear_axis":
                    if "ArrowUp" in keys:
                        item["value"] = min(item["upper"], item["value"] + item["step"])
                    if "ArrowDown" in keys:
                        item["value"] = max(item["lower"], item["value"] - item["step"])
                elif item["kind"] == "gripper" and item.get("arm") == active_arm:
                    if "KeyG" in keys:
                        item["aperture"] = 0.0 if item["aperture"] > 0.5 else 1.0

    def joint_values(self) -> Dict[str, float]:
        with self._lock:
            values: Dict[str, float] = {}
            for item in self._accessories.values():
                if item["kind"] == "linear_axis":
                    values[str(item["joint_name"])] = float(item["value"])
                elif item["kind"] == "gripper":
                    aperture = float(item["aperture"])
                    for idx, joint_name in enumerate(item["joint_names"]):
                        open_pos = float(item["open_positions"][idx])
                        closed_pos = float(item["closed_positions"][idx])
                        values[joint_name] = open_pos + aperture * (closed_pos - open_pos)
            return values

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            snapshot: Dict[str, Any] = {}
            for accessory_id, item in self._accessories.items():
                if item["kind"] == "linear_axis":
                    snapshot[accessory_id] = {
                        "kind": "linear_axis",
                        "label": item["label"],
                        "value": float(item["value"]),
                        "unit": "m",
                    }
                elif item["kind"] == "gripper":
                    aperture = float(item["aperture"])
                    snapshot[accessory_id] = {
                        "kind": "gripper",
                        "label": item["label"],
                        "arm": item.get("arm"),
                        "aperture": aperture,
                        "is_closed": aperture >= 0.5,
                    }
            return snapshot

    def describe(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [
                {
                    "id": accessory_id,
                    "kind": item["kind"],
                    "label": item["label"],
                    "arm": item.get("arm"),
                }
                for accessory_id, item in self._accessories.items()
            ]
