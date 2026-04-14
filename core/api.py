from __future__ import annotations

import math
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List

from core.fk import forward_kinematics
from core.ik import IKResult, solve_ik_dls
from core.robot import RobotModel
from core.state import SimulationState


class RMArmBridge:
    """Optional bridge to RM official API. Disabled by default."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.enabled = bool(config.get("enabled", False))
        self.connected = False
        self.error: str | None = None
        self._arm = None
        self._movej_kwargs: Dict[str, Any] = {}

        if not self.enabled:
            return

        try:
            api_root = Path(config.get("python_api_root", "third_party/RM_API2/Python")).resolve()
            if str(api_root) not in sys.path:
                sys.path.insert(0, str(api_root))

            from Robotic_Arm import rm_robot_interface as rm  # type: ignore

            thread_mode_name = str(config.get("thread_mode", "RM_DUAL_MODE_E"))
            thread_mode = getattr(rm.rm_thread_mode_e, thread_mode_name)

            arm = rm.RoboticArm(thread_mode)
            arm.rm_create_robot_arm(
                str(config["ip"]),
                int(config.get("port", 8080)),
                int(config.get("log_level", 3)),
            )

            self._movej_kwargs = {
                "v": int(config.get("speed_percent", 20)),
                "r": int(config.get("blend_percent", 0)),
                "connect": int(config.get("connect", 0)),
                "block": int(config.get("block", 0)),
            }

            self._arm = arm
            self.connected = True
        except Exception as exc:  # pragma: no cover - hardware branch
            self.connected = False
            self.enabled = False
            self.error = str(exc)

    def move_joint_radians(self, q_radians: List[float]) -> int | None:
        if not self.connected or self._arm is None:
            return None

        q_degrees = [math.degrees(value) for value in q_radians]
        return int(self._arm.rm_movej(q_degrees, **self._movej_kwargs))


class RobotAPI:
    """唯一控制入口：move_joint / move_ee / get_ee_pose"""

    def __init__(
        self,
        robot: RobotModel,
        state: SimulationState,
        ik_config: Dict[str, Any] | None = None,
        rm_bridge: RMArmBridge | None = None,
        q_init: List[float] | None = None,
    ) -> None:
        self.robot = robot
        self.state = state
        self.ik_config = ik_config or {}
        self.rm_bridge = rm_bridge
        self._lock = threading.RLock()
        self._q_init: List[float] = (
            list(q_init) if q_init is not None else list(state.q)
        )

    def _snapshot_locked(self) -> Dict[str, Any]:
        return {
            "q": self.state.q[:],
            "ee_pose": self.state.ee_pose[:],
            "joint_names": self.robot.joint_names[:],
            "base_link": self.robot.base_link,
            "ee_link": self.robot.ee_link,
        }

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return self._snapshot_locked()

    def move_joint(self, q: List[float]) -> Dict[str, Any]:
        with self._lock:
            q_next = self.robot.clamp_q(q)
            self.state.q = q_next
            self.state.ee_pose = forward_kinematics(self.robot, q_next)

            rm_ret = None
            if self.rm_bridge is not None:
                rm_ret = self.rm_bridge.move_joint_radians(q_next)

            out = self._snapshot_locked()
            out.update({"success": True, "rm_ret": rm_ret})
            return out

    def move_ee(self, pose: List[float]) -> Dict[str, Any]:
        if len(pose) != 6:
            raise ValueError("Target pose must have 6 values")

        with self._lock:
            result: IKResult = solve_ik_dls(
                self.robot,
                pose,
                self.state.q,
                damping=float(self.ik_config.get("damping", 0.12)),
                max_iterations=int(self.ik_config.get("max_iterations", 120)),
                tol_pos=float(self.ik_config.get("tol_pos", 1e-4)),
                tol_rot=float(self.ik_config.get("tol_rot", 1e-3)),
                max_step=float(self.ik_config.get("max_step", 0.12)),
                min_step=float(self.ik_config.get("min_step", 1e-6)),
                alpha=float(self.ik_config.get("alpha", 1.0)),
                orientation_weight=float(self.ik_config.get("orientation_weight", 0.5)),
            )

            self.state.q = self.robot.clamp_q(result.q)
            self.state.ee_pose = forward_kinematics(self.robot, self.state.q)

            rm_ret = None
            if self.rm_bridge is not None:
                rm_ret = self.rm_bridge.move_joint_radians(self.state.q)

            out = self._snapshot_locked()
            out.update(
                {
                    "success": result.success,
                    "ik": {
                        "iterations": result.iterations,
                        "position_error": result.position_error,
                        "orientation_error": result.orientation_error,
                    },
                    "rm_ret": rm_ret,
                }
            )
            return out

    def step_ee(self, delta_pose: List[float]) -> Dict[str, Any]:
        if len(delta_pose) != 6:
            raise ValueError("Delta pose must have 6 values")

        with self._lock:
            target_pose = [self.state.ee_pose[idx] + delta_pose[idx] for idx in range(6)]

        return self.move_ee(target_pose)

    def get_ee_pose(self) -> List[float]:
        with self._lock:
            return self.state.ee_pose[:]

    def home(self) -> Dict[str, Any]:
        with self._lock:
            return self.move_joint(self._q_init[:])

    def set_robot(
        self,
        robot: RobotModel,
        state: SimulationState,
        q_init: List[float],
    ) -> Dict[str, Any]:
        with self._lock:
            # ik_config and rm_bridge are intentionally preserved across arm swaps.
            self.robot = robot
            self.state = state
            self._q_init = list(q_init)
            return self._snapshot_locked()

