from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float

    def clamp(self, value: float) -> float:
        if value < self.lower:
            return self.lower
        if value > self.upper:
            return self.upper
        return value


@dataclass(frozen=True)
class KinematicSegment:
    name: str
    joint_type: str
    parent: str
    child: str
    axis: Vector3
    origin_xyz: Vector3
    origin_rpy: Vector3
    actuated: bool = False
    limit: JointLimit | None = None


@dataclass
class RobotModel:
    name: str
    urdf_path: str
    base_link: str
    ee_link: str
    segments: List[KinematicSegment]
    joint_names: List[str]
    joint_limits: Dict[str, JointLimit]
    _joint_index: Dict[str, int] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._joint_index = {name: idx for idx, name in enumerate(self.joint_names)}
        if len(self._joint_index) != len(self.joint_names):
            raise ValueError("Duplicate joint name found in joint_names")

        missing = [name for name in self.joint_names if name not in self.joint_limits]
        if missing:
            raise ValueError(f"Missing limits for joints: {missing}")

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    def joint_index(self, joint_name: str) -> int:
        return self._joint_index[joint_name]

    def zero_q(self) -> List[float]:
        return [0.0] * self.dof

    def clamp_joint(self, joint_name: str, value: float) -> float:
        return self.joint_limits[joint_name].clamp(value)

    def clamp_q(self, q: List[float]) -> List[float]:
        if len(q) != self.dof:
            raise ValueError(f"Expected {self.dof} joints, got {len(q)}")

        return [
            self.joint_limits[self.joint_names[idx]].clamp(value)
            for idx, value in enumerate(q)
        ]

