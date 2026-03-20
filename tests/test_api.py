from __future__ import annotations

import math

from core.api import RobotAPI
from core.fk import forward_kinematics
from core.state import make_initial_state
from tests.helpers import make_planar_robot


def _dist3(a: list[float], b: list[float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def test_move_joint_clamps_limits() -> None:
    robot = make_planar_robot()
    api = RobotAPI(robot=robot, state=make_initial_state(robot))

    out = api.move_joint([10.0, -10.0])
    assert abs(out["q"][0] - math.pi) < 1e-6
    assert abs(out["q"][1] + math.pi) < 1e-6


def test_move_ee_updates_state() -> None:
    robot = make_planar_robot()
    api = RobotAPI(robot=robot, state=make_initial_state(robot))

    q_target = [0.2, -0.3]
    pose_target = forward_kinematics(robot, q_target)
    out = api.move_ee(pose_target)

    assert out["success"]
    assert _dist3(out["ee_pose"], pose_target) < 1e-3

