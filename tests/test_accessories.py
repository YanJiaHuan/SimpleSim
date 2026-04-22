from __future__ import annotations

from core.accessories import AccessoryController


def test_accessory_controller_reports_semantic_snapshot_and_joint_values() -> None:
    controller = AccessoryController(
        {
            "accessories": [
                {
                    "id": "column",
                    "kind": "linear_axis",
                    "label": "Elevation",
                    "joint_name": "elevation",
                    "lower": -0.8,
                    "upper": 0.0,
                    "init": -0.3,
                    "step": 0.005,
                },
                {
                    "id": "right_gripper",
                    "kind": "gripper",
                    "label": "Right Grip",
                    "arm": "right",
                    "joint_names": ["finger_a", "finger_b"],
                    "open_positions": [0.04, 0.04],
                    "closed_positions": [0.0, 0.0],
                },
            ]
        }
    )

    controller.apply_keys(["ArrowUp"], active_arm="right")
    controller.apply_keys(["KeyG"], active_arm="right")

    snapshot = controller.snapshot()
    joint_values = controller.joint_values()

    assert snapshot["column"]["kind"] == "linear_axis"
    assert snapshot["column"]["value"] == -0.295
    assert snapshot["right_gripper"]["kind"] == "gripper"
    assert snapshot["right_gripper"]["is_closed"] is True
    assert joint_values["elevation"] == -0.295
    assert joint_values["finger_a"] == 0.0
    assert joint_values["finger_b"] == 0.0
