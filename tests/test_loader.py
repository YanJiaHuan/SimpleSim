from __future__ import annotations

import json
from pathlib import Path

from loader.urdf import load_robot_model


def test_urdf_load_left_arm_chain() -> None:
    robot = load_robot_model(
        urdf_path="third_party/TR4_Pro/TR4_with_grippers_v2.urdf",
        base_link="left_arm_base_link",
        ee_link="left_gripper_base_link",
        expected_joint_names=[
            "left_arm_joint1",
            "left_arm_joint2",
            "left_arm_joint3",
            "left_arm_joint4",
            "left_arm_joint5",
            "left_arm_joint6",
        ],
    )

    assert robot.dof == 6
    assert robot.base_link == "left_arm_base_link"
    assert robot.ee_link == "left_gripper_base_link"
    assert len(robot.segments) == 7


def test_urdf_load_fr3_chain() -> None:
    robot = load_robot_model(
        urdf_path="third_party/franka_description/simplesim/fr3_with_franka_hand.urdf",
        base_link="fr3_link0",
        ee_link="fr3_hand_tcp",
        expected_joint_names=[
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ],
    )

    assert robot.dof == 7
    assert robot.base_link == "fr3_link0"
    assert robot.ee_link == "fr3_hand_tcp"


def test_fr3_config_declares_single_arm_hand_accessory() -> None:
    config = json.loads(Path("configs/fr3.yaml").read_text(encoding="utf-8"))

    assert config["active_arm"] == "main"
    assert list(config["arms"].keys()) == ["main"]
    assert config["accessories"] == [
        {
            "id": "hand",
            "kind": "gripper",
            "label": "Hand",
            "arm": "main",
            "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
            "open_positions": [0.04, 0.04],
            "closed_positions": [0.0, 0.0],
        }
    ]
