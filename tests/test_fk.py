from __future__ import annotations

import math

from core.fk import forward_kinematics
from tests.helpers import make_planar_robot


def _close(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(a - b) <= tol


def test_fk_planar_zero_pose() -> None:
    robot = make_planar_robot()
    pose = forward_kinematics(robot, [0.0, 0.0])
    assert _close(pose[0], 2.0)
    assert _close(pose[1], 0.0)


def test_fk_planar_quadrant_pose() -> None:
    robot = make_planar_robot()
    pose = forward_kinematics(robot, [math.pi / 2.0, 0.0])
    assert _close(pose[0], 0.0, 1e-5)
    assert _close(pose[1], 2.0, 1e-5)

