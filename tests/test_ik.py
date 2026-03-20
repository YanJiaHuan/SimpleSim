from __future__ import annotations

from core.fk import forward_kinematics
from core.ik import solve_ik_dls
from tests.helpers import make_planar_robot


def _dist3(a: list[float], b: list[float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def test_ik_round_trip_fk_ik_fk() -> None:
    robot = make_planar_robot()
    q_target = [0.4, -0.7]
    pose_target = forward_kinematics(robot, q_target)

    result = solve_ik_dls(
        robot=robot,
        target_pose=pose_target,
        q_seed=[0.0, 0.0],
        damping=0.08,
        max_iterations=300,
        tol_pos=1e-5,
        tol_rot=1e-4,
        max_step=0.15,
    )

    assert result.success
    pose_solved = forward_kinematics(robot, result.q)
    assert _dist3(pose_solved, pose_target) < 1e-3

