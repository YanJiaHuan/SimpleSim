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


def test_home_returns_to_q_init() -> None:
    robot = make_planar_robot()
    q_init = [0.1, -0.1]
    state = make_initial_state(robot, q_init)
    api = RobotAPI(robot=robot, state=state, q_init=q_init)

    api.move_joint([0.5, 0.6])
    out = api.home()

    assert out["success"]
    assert abs(out["q"][0] - 0.1) < 1e-9
    assert abs(out["q"][1] + 0.1) < 1e-9


def test_set_robot_swaps_model_and_q_init() -> None:
    robot_a = make_planar_robot()
    api = RobotAPI(
        robot=robot_a,
        state=make_initial_state(robot_a, [0.3, 0.4]),
        q_init=[0.3, 0.4],
    )
    assert api.robot is robot_a

    robot_b = make_planar_robot()
    new_q_init = [0.0, 0.0]
    api.set_robot(
        robot=robot_b,
        state=make_initial_state(robot_b, new_q_init),
        q_init=new_q_init,
    )

    assert api.robot is robot_b

    # Push the swapped robot off-home so home() has something to undo.
    api.move_joint([0.25, -0.25])
    assert abs(api.state.q[0] - 0.25) < 1e-9
    assert abs(api.state.q[1] + 0.25) < 1e-9

    out = api.home()
    assert out["q"] == [0.0, 0.0]
    assert api.state.q == [0.0, 0.0]

