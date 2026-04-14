from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from core.robot import KinematicSegment, RobotModel


Matrix4 = List[List[float]]
Vector3 = Tuple[float, float, float]
Vector6 = List[float]


@dataclass(frozen=True)
class JointFrame:
    name: str
    joint_type: str
    position: Vector3
    axis_world: Vector3


def _identity4() -> Matrix4:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_mul(a: Matrix4, b: Matrix4) -> Matrix4:
    out = [[0.0] * 4 for _ in range(4)]
    for r in range(4):
        for c in range(4):
            out[r][c] = (
                a[r][0] * b[0][c]
                + a[r][1] * b[1][c]
                + a[r][2] * b[2][c]
                + a[r][3] * b[3][c]
            )
    return out


def _norm3(v: Vector3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _normalize3(v: Vector3) -> Vector3:
    n = _norm3(v)
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _transform_from_xyz_rpy(xyz: Vector3, rpy: Vector3) -> Matrix4:
    r = _rotation_from_rpy(rpy[0], rpy[1], rpy[2])
    return [
        [r[0][0], r[0][1], r[0][2], xyz[0]],
        [r[1][0], r[1][1], r[1][2], xyz[1]],
        [r[2][0], r[2][1], r[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_axis_angle(axis: Vector3, angle: float) -> List[List[float]]:
    x, y, z = _normalize3(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c

    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ]


def _motion_transform(joint_type: str, axis: Vector3, value: float) -> Matrix4:
    if joint_type in ("revolute", "continuous"):
        r = _rotation_axis_angle(axis, value)
        return [
            [r[0][0], r[0][1], r[0][2], 0.0],
            [r[1][0], r[1][1], r[1][2], 0.0],
            [r[2][0], r[2][1], r[2][2], 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    if joint_type == "prismatic":
        ax = _normalize3(axis)
        return [
            [1.0, 0.0, 0.0, ax[0] * value],
            [0.0, 1.0, 0.0, ax[1] * value],
            [0.0, 0.0, 1.0, ax[2] * value],
            [0.0, 0.0, 0.0, 1.0],
        ]

    return _identity4()


def _rotate_vector(transform: Matrix4, vec: Vector3) -> Vector3:
    return (
        transform[0][0] * vec[0] + transform[0][1] * vec[1] + transform[0][2] * vec[2],
        transform[1][0] * vec[0] + transform[1][1] * vec[1] + transform[1][2] * vec[2],
        transform[2][0] * vec[0] + transform[2][1] * vec[1] + transform[2][2] * vec[2],
    )


def _translation(transform: Matrix4) -> Vector3:
    return (transform[0][3], transform[1][3], transform[2][3])


def pose_to_transform(pose: Vector6) -> Matrix4:
    if len(pose) != 6:
        raise ValueError("Pose must be [x, y, z, roll, pitch, yaw]")
    return _transform_from_xyz_rpy(
        (float(pose[0]), float(pose[1]), float(pose[2])),
        (float(pose[3]), float(pose[4]), float(pose[5])),
    )


def transform_to_pose(transform: Matrix4) -> Vector6:
    r00, r01, r02 = transform[0][0], transform[0][1], transform[0][2]
    r10, r11, r12 = transform[1][0], transform[1][1], transform[1][2]
    r20, r21, r22 = transform[2][0], transform[2][1], transform[2][2]

    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    cp = math.cos(pitch)

    if abs(cp) > 1e-9:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        yaw = 0.0

    return [
        transform[0][3],
        transform[1][3],
        transform[2][3],
        roll,
        pitch,
        yaw,
    ]


def forward_kinematics_matrix(
    robot: RobotModel,
    q: List[float],
    return_joint_frames: bool = False,
) -> Matrix4 | tuple[Matrix4, List[JointFrame]]:
    q_clamped = robot.clamp_q(q)
    q_by_name = {name: q_clamped[idx] for idx, name in enumerate(robot.joint_names)}

    # Start in world frame if the loader provided the base_link's world transform.
    if robot.base_world_transform is not None:
        transform = [row[:] for row in robot.base_world_transform]
    else:
        transform = _identity4()
    joint_frames: List[JointFrame] = []

    for segment in robot.segments:
        t_origin = _transform_from_xyz_rpy(segment.origin_xyz, segment.origin_rpy)
        t_joint = _mat4_mul(transform, t_origin)

        if segment.actuated:
            axis_world = _normalize3(_rotate_vector(t_joint, segment.axis))
            joint_frames.append(
                JointFrame(
                    name=segment.name,
                    joint_type=segment.joint_type,
                    position=_translation(t_joint),
                    axis_world=axis_world,
                )
            )
            value = q_by_name[segment.name]
            t_motion = _motion_transform(segment.joint_type, segment.axis, value)
            transform = _mat4_mul(t_joint, t_motion)
        else:
            transform = t_joint

    if return_joint_frames:
        return transform, joint_frames

    return transform


def forward_kinematics(robot: RobotModel, q: List[float]) -> Vector6:
    return transform_to_pose(forward_kinematics_matrix(robot, q))


def _segment_to_string(segment: KinematicSegment) -> str:
    return f"{segment.name}({segment.parent}->{segment.child}, {segment.joint_type})"

