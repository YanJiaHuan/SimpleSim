from __future__ import annotations

import math
from pathlib import Path

from core.accessories import AccessoryController
from core.api import RobotAPI
from core.fk import forward_kinematics
from core.state import make_initial_state
from interface.keyboard import KeyboardInterface
from renderer.viewer import RuntimeServer
from tests.helpers import make_planar_robot


def _dist3(a: list[float], b: list[float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def test_move_joint_clamps_limits() -> None:
    robot = make_planar_robot()
    api = RobotAPI(robot=robot, state=make_initial_state(robot))

    out = api.move_joint([10.0, -10.0])
    assert abs(out["q"][0] - math.pi) < 1e-6
    assert abs(out["q"][1] + math.pi) < 1e-6


def test_move_joint_response_has_no_rm_bridge_fields() -> None:
    robot = make_planar_robot()
    api = RobotAPI(robot=robot, state=make_initial_state(robot))

    out = api.move_joint([0.2, -0.2])

    assert out["success"] is True
    assert "rm_ret" not in out


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

    api.move_joint([0.25, -0.25])
    assert abs(api.state.q[0] - 0.25) < 1e-9
    assert abs(api.state.q[1] + 0.25) < 1e-9

    out = api.home()
    assert out["q"] == [0.0, 0.0]
    assert api.state.q == [0.0, 0.0]


def test_runtime_meta_reports_capabilities_from_config() -> None:
    robot = make_planar_robot()
    server = RuntimeServer(
        api=RobotAPI(robot=robot, state=make_initial_state(robot)),
        keyboard=KeyboardInterface(),
        accessory=AccessoryController(
            {
                "accessories": [
                    {
                        "id": "hand",
                        "kind": "gripper",
                        "label": "Hand",
                        "arm": "main",
                        "joint_names": ["finger_joint1", "finger_joint2"],
                        "open_positions": [0.04, 0.04],
                        "closed_positions": [0.0, 0.0],
                    }
                ]
            }
        ),
        static_root=Path("."),
        host="127.0.0.1",
        port=8765,
        urdf_url="/robot.urdf",
        arms_data={"main": (robot, make_initial_state(robot), [0.0, 0.0])},
        config={
            "robot_label": "Planar",
            "arms": {
                "main": {
                    "label": "Main Arm",
                    "base_link": "base",
                    "ee_link": "ee",
                    "joint_names": ["joint1", "joint2"],
                    "q_init": [0.0, 0.0],
                }
            },
        },
        active_arm="main",
    )

    meta = server._meta_payload()

    assert meta["controls"]["can_switch_arm"] is False
    assert meta["controls"]["show_linear_axis_controls"] is False
    assert meta["panels"]["show_arm_badge"] is False
    assert meta["panels"]["accessories"] == [
        {"id": "hand", "kind": "gripper", "label": "Hand", "arm": "main"}
    ]


def test_runtime_meta_reports_arm_switch_for_multi_arm_only() -> None:
    robot = make_planar_robot()
    single_arm_server = RuntimeServer(
        api=RobotAPI(robot=robot, state=make_initial_state(robot)),
        keyboard=KeyboardInterface(),
        accessory=AccessoryController({"accessories": []}),
        static_root=Path("."),
        host="127.0.0.1",
        port=8765,
        urdf_url="/robot.urdf",
        arms_data={"main": (robot, make_initial_state(robot), [0.0, 0.0])},
        config={"arms": {"main": {"label": "Main", "base_link": "base", "ee_link": "ee", "joint_names": ["joint1", "joint2"], "q_init": [0.0, 0.0]}}},
        active_arm="main",
    )
    dual_arm_server = RuntimeServer(
        api=RobotAPI(robot=robot, state=make_initial_state(robot)),
        keyboard=KeyboardInterface(),
        accessory=AccessoryController({"accessories": []}),
        static_root=Path("."),
        host="127.0.0.1",
        port=8765,
        urdf_url="/robot.urdf",
        arms_data={
            "left": (robot, make_initial_state(robot), [0.0, 0.0]),
            "right": (robot, make_initial_state(robot), [0.0, 0.0]),
        },
        config={
            "arms": {
                "left": {"label": "Left", "base_link": "base", "ee_link": "ee", "joint_names": ["joint1", "joint2"], "q_init": [0.0, 0.0]},
                "right": {"label": "Right", "base_link": "base", "ee_link": "ee", "joint_names": ["joint1", "joint2"], "q_init": [0.0, 0.0]},
            }
        },
        active_arm="left",
    )

    assert single_arm_server._meta_payload()["controls"]["can_switch_arm"] is False
    assert dual_arm_server._meta_payload()["controls"]["can_switch_arm"] is True
