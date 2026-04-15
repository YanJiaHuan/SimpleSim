from __future__ import annotations

import threading
from typing import Any, Dict, List


class AccessoryController:
    """Manages elevator and gripper joints outside the IK arm chain.

    Elevator
    --------
    Single prismatic joint.  ArrowUp/Down step the position within [lower, upper].

    Gripper
    -------
    Each arm has two 3-link fingers (l_joint1-3 and r_joint1-3).  A single
    ``aperture`` parameter in [0=open, 1=closed] drives all six joints via
    per-joint coupling coefficients.

    The coupling list must have one entry per joint_name.  Positive coupling
    closes the finger, negative opens it.  Magnitude scales the max_angle, so
    coupling=1.0 means the joint travels its full max_angle when aperture=1.

    Example (symmetric parallel gripper):
        joint_names = [l_j1, l_j2, l_j3, r_j1, r_j2, r_j3]
        coupling    = [-1.0, -1.0, -1.0,  1.0,  1.0,  1.0]

    The left-finger joints rotate in the negative direction to close; the
    right-finger joints in the positive direction.  Because all joints share
    the same axis (0 0 -1) in the URDF, this produces symmetric closure.
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        self._lock = threading.RLock()

        elev = config.get("elevator", {})
        self._elev_joint: str = str(elev.get("joint_name", "elevation"))
        self._elev_lower: float = float(elev.get("lower", 0.0))
        self._elev_upper: float = float(elev.get("upper", 0.35))
        self._elev_step: float = float(elev.get("step", 0.01))
        self._elev_pos: float = max(
            self._elev_lower,
            min(self._elev_upper, float(elev.get("init", self._elev_lower))),
        )

        gripper_cfg: Dict[str, Any] = config.get("grippers", {})
        # Per-arm: joint names, coupling coefficients, max_angle, step, current aperture
        self._grippers: Dict[str, Dict[str, Any]] = {}
        for arm_name, gcfg in gripper_cfg.items():
            joint_names = [str(n) for n in gcfg.get("joint_names", [])]
            n = len(joint_names)
            self._grippers[arm_name] = {
                "joint_names": joint_names,
                "open_positions":   [float(v) for v in gcfg.get("open_positions",   [0.0] * n)],
                "closed_positions": [float(v) for v in gcfg.get("closed_positions", [0.0] * n)],
                "aperture": 0.0,
            }

    # ── Elevator ──────────────────────────────────────────────────────────────

    def step_elevator(self, direction: int) -> None:
        """direction: +1 = up, -1 = down."""
        with self._lock:
            self._elev_pos = max(
                self._elev_lower,
                min(self._elev_upper, self._elev_pos + direction * self._elev_step),
            )

    # ── Gripper ───────────────────────────────────────────────────────────────

    def toggle_gripper(self, arm_name: str) -> None:
        """Toggle gripper between fully open (aperture=0) and fully closed (aperture=1)."""
        with self._lock:
            g = self._grippers.get(arm_name)
            if g is None:
                return
            g["aperture"] = 0.0 if g["aperture"] > 0.5 else 1.0

    # ── Joint values (for renderer / Three.js) ────────────────────────────────

    def joint_values(self) -> Dict[str, float]:
        """Return {joint_name: value} for every accessory joint."""
        with self._lock:
            vals: Dict[str, float] = {self._elev_joint: self._elev_pos}
            for g in self._grippers.values():
                aperture = g["aperture"]
                open_pos = g["open_positions"]
                clos_pos = g["closed_positions"]
                for i, jname in enumerate(g["joint_names"]):
                    vals[jname] = open_pos[i] + aperture * (clos_pos[i] - open_pos[i])
            return vals

    # ── Snapshot (for API responses) ──────────────────────────────────────────

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "elevator": self._elev_pos,
                "grippers": {
                    arm: g["aperture"] for arm, g in self._grippers.items()
                },
            }
