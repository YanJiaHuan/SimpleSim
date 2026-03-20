from __future__ import annotations

from loader.urdf import load_robot_model


def test_urdf_load_left_arm_chain() -> None:
    robot = load_robot_model(
        urdf_path="TR4_Pro/TR4_with_grippers_v2.urdf",
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

