from __future__ import annotations

import threading
from typing import Any, Dict, List

from core.fk import forward_kinematics
from core.ik import IKResult, solve_ik_dls
from core.robot import RobotModel
from core.state import SimulationState


class RobotAPI:
    """唯一控制入口：move_joint / move_ee / get_ee_pose"""

    def __init__(
        self,
        robot: RobotModel,
        state: SimulationState,
        ik_config: Dict[str, Any] | None = None,
        q_init: List[float] | None = None,
    ) -> None:
        self.robot = robot
        self.state = state
        self.ik_config = ik_config or {}
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

            out = self._snapshot_locked()
            out.update({"success": True})
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

            out = self._snapshot_locked()
            out.update(
                {
                    "success": result.success,
                    "ik": {
                        "iterations": result.iterations,
                        "position_error": result.position_error,
                        "orientation_error": result.orientation_error,
                    },
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
            self.robot = robot
            self.state = state
            self._q_init = list(q_init)
            return self._snapshot_locked()
