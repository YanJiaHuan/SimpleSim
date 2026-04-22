from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_expected_robot_asset_entrypoints_exist() -> None:
    assert (ROOT / "third_party" / "TR4_Pro" / "TR4_with_grippers_v2.urdf").is_file()
    assert (
        ROOT
        / "third_party"
        / "franka_description"
        / "simplesim"
        / "fr3_with_franka_hand.urdf"
    ).is_file()


def test_gitmodules_contains_franka_and_not_rm_api2() -> None:
    gitmodules = (ROOT / ".gitmodules").read_text(encoding="utf-8")
    assert '[submodule "third_party/franka_description"]' in gitmodules
    assert '[submodule "third_party/RM_API2"]' not in gitmodules
