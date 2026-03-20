from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from core.fk import (
    forward_kinematics_matrix,
    pose_to_transform,
    transform_to_pose,
)
from core.robot import RobotModel


Matrix = List[List[float]]
Vector = List[float]
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: List[float]
    iterations: int
    position_error: float
    orientation_error: float


def _sub3(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vector) -> float:
    return math.sqrt(sum(x * x for x in v))


def _transpose(m: Matrix) -> Matrix:
    rows = len(m)
    cols = len(m[0]) if rows else 0
    return [[m[r][c] for r in range(rows)] for c in range(cols)]


def _mat_mul(a: Matrix, b: Matrix) -> Matrix:
    rows = len(a)
    cols = len(b[0]) if b else 0
    inner = len(b)
    out = [[0.0] * cols for _ in range(rows)]
    for r in range(rows):
        for c in range(cols):
            val = 0.0
            for k in range(inner):
                val += a[r][k] * b[k][c]
            out[r][c] = val
    return out


def _mat_vec_mul(m: Matrix, v: Vector) -> Vector:
    return [sum(row[i] * v[i] for i in range(len(v))) for row in m]


def _solve_linear_system(a: Matrix, b: Vector) -> Vector:
    n = len(a)
    aug = [a[r][:] + [b[r]] for r in range(n)]

    for col in range(n):
        pivot = col
        pivot_abs = abs(aug[col][col])
        for row in range(col + 1, n):
            val = abs(aug[row][col])
            if val > pivot_abs:
                pivot = row
                pivot_abs = val

        if pivot_abs < 1e-12:
            raise ValueError("Singular matrix in DLS solve")

        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]

        pivot_val = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot_val

        for row in range(col + 1, n):
            factor = aug[row][col]
            if abs(factor) < 1e-15:
                continue
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]

    x = [0.0] * n
    for row in range(n - 1, -1, -1):
        rhs = aug[row][n]
        for col in range(row + 1, n):
            rhs -= aug[row][col] * x[col]
        x[row] = rhs
    return x


def _pose_error(current_t: Matrix, target_t: Matrix) -> Vector:
    p_cur = (current_t[0][3], current_t[1][3], current_t[2][3])
    p_tgt = (target_t[0][3], target_t[1][3], target_t[2][3])
    pos_err = [p_tgt[0] - p_cur[0], p_tgt[1] - p_cur[1], p_tgt[2] - p_cur[2]]

    rc0 = (current_t[0][0], current_t[1][0], current_t[2][0])
    rc1 = (current_t[0][1], current_t[1][1], current_t[2][1])
    rc2 = (current_t[0][2], current_t[1][2], current_t[2][2])
    rt0 = (target_t[0][0], target_t[1][0], target_t[2][0])
    rt1 = (target_t[0][1], target_t[1][1], target_t[2][1])
    rt2 = (target_t[0][2], target_t[1][2], target_t[2][2])

    c0 = _cross(rc0, rt0)
    c1 = _cross(rc1, rt1)
    c2 = _cross(rc2, rt2)
    rot_err = [
        0.5 * (c0[0] + c1[0] + c2[0]),
        0.5 * (c0[1] + c1[1] + c2[1]),
        0.5 * (c0[2] + c1[2] + c2[2]),
    ]

    return pos_err + rot_err


def _geometric_jacobian(robot: RobotModel, q: List[float]) -> tuple[Matrix, Matrix]:
    t_ee, joint_frames = forward_kinematics_matrix(robot, q, return_joint_frames=True)
    p_ee = (t_ee[0][3], t_ee[1][3], t_ee[2][3])
    frame_by_name = {frame.name: frame for frame in joint_frames}

    jacobian = [[0.0] * robot.dof for _ in range(6)]
    for col, joint_name in enumerate(robot.joint_names):
        frame = frame_by_name[joint_name]
        axis = frame.axis_world

        if frame.joint_type in ("revolute", "continuous"):
            linear = _cross(axis, _sub3(p_ee, frame.position))
            angular = axis
        else:
            linear = axis
            angular = (0.0, 0.0, 0.0)

        jacobian[0][col] = linear[0]
        jacobian[1][col] = linear[1]
        jacobian[2][col] = linear[2]
        jacobian[3][col] = angular[0]
        jacobian[4][col] = angular[1]
        jacobian[5][col] = angular[2]

    return jacobian, t_ee


def _dls_step(jacobian: Matrix, error: Vector, damping: float) -> Vector:
    jt = _transpose(jacobian)
    jjt = _mat_mul(jacobian, jt)
    damp2 = damping * damping
    for idx in range(len(jjt)):
        jjt[idx][idx] += damp2
    y = _solve_linear_system(jjt, error)
    return _mat_vec_mul(jt, y)


def solve_ik_dls(
    robot: RobotModel,
    target_pose: List[float],
    q_seed: List[float],
    damping: float = 0.12,
    max_iterations: int = 100,
    tol_pos: float = 1e-4,
    tol_rot: float = 1e-3,
    max_step: float = 0.12,
    min_step: float = 1e-6,
    alpha: float = 1.0,
    orientation_weight: float = 0.5,
) -> IKResult:
    target_t = pose_to_transform(target_pose)
    q = robot.clamp_q(q_seed)

    best_q = q[:]
    best_score = float("inf")
    best_pos_err = float("inf")
    best_rot_err = float("inf")

    for iteration in range(1, max_iterations + 1):
        jacobian, current_t = _geometric_jacobian(robot, q)
        err = _pose_error(current_t, target_t)
        pos_err = _norm(err[:3])
        rot_err = _norm(err[3:])

        score = pos_err + orientation_weight * rot_err
        if score < best_score:
            best_score = score
            best_q = q[:]
            best_pos_err = pos_err
            best_rot_err = rot_err

        if pos_err < tol_pos and rot_err < tol_rot:
            return IKResult(
                success=True,
                q=q[:],
                iterations=iteration,
                position_error=pos_err,
                orientation_error=rot_err,
            )

        weighted_err = err[:3] + [orientation_weight * value for value in err[3:]]
        weighted_j = [row[:] for row in jacobian]
        for row in (3, 4, 5):
            for col in range(robot.dof):
                weighted_j[row][col] *= orientation_weight

        try:
            dq = _dls_step(weighted_j, weighted_err, damping)
        except ValueError:
            break

        max_abs = max(abs(v) for v in dq) if dq else 0.0
        if max_abs < min_step:
            break
        if max_abs > max_step:
            scale = max_step / max_abs
            dq = [v * scale for v in dq]

        q_next = robot.clamp_q([q[idx] + alpha * dq[idx] for idx in range(robot.dof)])
        if max(abs(q_next[idx] - q[idx]) for idx in range(robot.dof)) < min_step:
            q = q_next
            break

        q = q_next

    final_pose = transform_to_pose(forward_kinematics_matrix(robot, best_q))
    final_t = pose_to_transform(final_pose)
    final_err = _pose_error(final_t, target_t)

    return IKResult(
        success=False,
        q=best_q,
        iterations=max_iterations,
        position_error=_norm(final_err[:3]),
        orientation_error=_norm(final_err[3:]),
    )

