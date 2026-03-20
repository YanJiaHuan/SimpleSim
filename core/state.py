from __future__ import annotations

from dataclasses import dataclass
from typing import List

from core.fk import forward_kinematics
from core.robot import RobotModel


@dataclass
class SimulationState:
    q: List[float]
    ee_pose: List[float]


def make_initial_state(robot: RobotModel, q_init: List[float] | None = None) -> SimulationState:
    q = robot.zero_q() if q_init is None else robot.clamp_q(q_init)
    return SimulationState(q=q, ee_pose=forward_kinematics(robot, q))

