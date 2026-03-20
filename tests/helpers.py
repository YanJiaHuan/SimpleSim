from __future__ import annotations

import math

from core.robot import JointLimit, KinematicSegment, RobotModel


def make_planar_robot() -> RobotModel:
    segments = [
        KinematicSegment(
            name="joint1",
            joint_type="revolute",
            parent="base",
            child="link1",
            axis=(0.0, 0.0, 1.0),
            origin_xyz=(0.0, 0.0, 0.0),
            origin_rpy=(0.0, 0.0, 0.0),
            actuated=True,
            limit=JointLimit(-math.pi, math.pi),
        ),
        KinematicSegment(
            name="joint2",
            joint_type="revolute",
            parent="link1",
            child="link2",
            axis=(0.0, 0.0, 1.0),
            origin_xyz=(1.0, 0.0, 0.0),
            origin_rpy=(0.0, 0.0, 0.0),
            actuated=True,
            limit=JointLimit(-math.pi, math.pi),
        ),
        KinematicSegment(
            name="tool_fixed",
            joint_type="fixed",
            parent="link2",
            child="ee",
            axis=(0.0, 0.0, 1.0),
            origin_xyz=(1.0, 0.0, 0.0),
            origin_rpy=(0.0, 0.0, 0.0),
            actuated=False,
            limit=None,
        ),
    ]

    return RobotModel(
        name="planar",
        urdf_path="/tmp/planar.urdf",
        base_link="base",
        ee_link="ee",
        segments=segments,
        joint_names=["joint1", "joint2"],
        joint_limits={
            "joint1": JointLimit(-math.pi, math.pi),
            "joint2": JointLimit(-math.pi, math.pi),
        },
    )

