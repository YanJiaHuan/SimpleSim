from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

from core.robot import JointLimit, KinematicSegment, RobotModel


def _parse_vec3(raw: str | None, fallback: Tuple[float, float, float]) -> Tuple[float, float, float]:
    if not raw:
        return fallback
    parts = raw.strip().split()
    if len(parts) != 3:
        raise ValueError(f"Expected vec3, got: {raw}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def _find_path(
    parent_to_joints: Dict[str, List[str]],
    joints: Dict[str, KinematicSegment],
    current_link: str,
    target_link: str,
    path: List[str],
) -> List[str] | None:
    if current_link == target_link:
        return path[:]

    for joint_name in parent_to_joints.get(current_link, []):
        child_link = joints[joint_name].child
        result = _find_path(
            parent_to_joints=parent_to_joints,
            joints=joints,
            current_link=child_link,
            target_link=target_link,
            path=path + [joint_name],
        )
        if result is not None:
            return result

    return None


def _limit_from_joint(joint_type: str, limit_node: ET.Element | None) -> JointLimit | None:
    if joint_type == "fixed":
        return None

    if joint_type == "continuous":
        return JointLimit(lower=-math.pi, upper=math.pi)

    if limit_node is None:
        if joint_type == "prismatic":
            return JointLimit(lower=-1.0, upper=1.0)
        return JointLimit(lower=-math.pi, upper=math.pi)

    lower = float(limit_node.attrib.get("lower", "-3.141592653589793"))
    upper = float(limit_node.attrib.get("upper", "3.141592653589793"))
    return JointLimit(lower=lower, upper=upper)


def load_robot_model(
    urdf_path: str,
    base_link: str,
    ee_link: str,
    expected_joint_names: List[str] | None = None,
) -> RobotModel:
    urdf_abs = Path(urdf_path).resolve()
    root = ET.parse(urdf_abs).getroot()

    joints: Dict[str, KinematicSegment] = {}
    parent_to_joints: Dict[str, List[str]] = {}

    for joint_node in root.findall("joint"):
        name = joint_node.attrib["name"]
        joint_type = joint_node.attrib["type"]
        parent = joint_node.find("parent")
        child = joint_node.find("child")
        if parent is None or child is None:
            continue

        origin = joint_node.find("origin")
        axis_node = joint_node.find("axis")
        limit_node = joint_node.find("limit")

        origin_xyz = _parse_vec3(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
        origin_rpy = _parse_vec3(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
        axis = _parse_vec3(axis_node.attrib.get("xyz") if axis_node is not None else None, (0.0, 0.0, 1.0))
        actuated = joint_type in ("revolute", "continuous", "prismatic")
        limit = _limit_from_joint(joint_type, limit_node)

        segment = KinematicSegment(
            name=name,
            joint_type=joint_type,
            parent=parent.attrib["link"],
            child=child.attrib["link"],
            axis=axis,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            actuated=actuated,
            limit=limit,
        )
        joints[name] = segment
        parent_to_joints.setdefault(segment.parent, []).append(name)

    path_joint_names = _find_path(
        parent_to_joints=parent_to_joints,
        joints=joints,
        current_link=base_link,
        target_link=ee_link,
        path=[],
    )
    if path_joint_names is None:
        raise ValueError(f"Cannot find chain path from {base_link} to {ee_link}")

    segments = [joints[name] for name in path_joint_names]
    actuated_joint_names = [seg.name for seg in segments if seg.actuated]

    if expected_joint_names is not None and actuated_joint_names != expected_joint_names:
        raise ValueError(
            "Actuated joint list from URDF path does not match config. "
            f"from_urdf={actuated_joint_names}, expected={expected_joint_names}"
        )

    joint_limits = {
        seg.name: seg.limit
        for seg in segments
        if seg.actuated and seg.limit is not None
    }

    return RobotModel(
        name=root.attrib.get("name", "robot"),
        urdf_path=str(urdf_abs),
        base_link=base_link,
        ee_link=ee_link,
        segments=segments,
        joint_names=actuated_joint_names,
        joint_limits=joint_limits,
    )

