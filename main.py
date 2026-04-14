from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict

from core.api import RMArmBridge, RobotAPI
from core.state import make_initial_state
from interface.keyboard import KeyboardInterface
from loader.urdf import load_robot_model
from renderer.viewer import make_server


def _load_config(config_path: Path) -> Dict[str, Any]:
    raw = config_path.read_text(encoding="utf-8")
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(
            "Config must be JSON-formatted YAML (YAML superset)."
        ) from exc


def _to_url_path(path: Path, root: Path) -> str:
    rel = path.resolve().relative_to(root.resolve())
    return "/" + rel.as_posix()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SimpleSim robot runtime")
    parser.add_argument(
        "--urdf",
        default="TR4_Pro/TR4_with_grippers_v2.urdf",
        help="Path to URDF file (default: TR4 Pro with grippers)",
    )
    parser.add_argument(
        "--config",
        default="configs/tr4.yaml",
        help="Path to config file (default: configs/tr4.yaml)",
    )
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    project_root = Path(__file__).resolve().parent
    urdf_path = Path(args.urdf).resolve()
    config_path = Path(args.config).resolve()

    config = _load_config(config_path)
    active_arm_name = str(config.get("active_arm", "left"))
    arm_config = config.get("arms", {}).get(active_arm_name)
    if arm_config is None:
        raise ValueError(f"No arm config named '{active_arm_name}'")

    robot = load_robot_model(
        urdf_path=str(urdf_path),
        base_link=str(arm_config["base_link"]),
        ee_link=str(arm_config["ee_link"]),
        expected_joint_names=[str(name) for name in arm_config.get("joint_names", [])],
    )

    state = make_initial_state(robot, [float(v) for v in arm_config.get("q_init", [0.0] * robot.dof)])
    rm_bridge = RMArmBridge(config.get("rm_api", {}))
    api = RobotAPI(robot=robot, state=state, ik_config=config.get("ik", {}), rm_bridge=rm_bridge)

    keyboard = KeyboardInterface(
        translation_step=float(config.get("keyboard", {}).get("translation_step", 0.01)),
        rotation_step=float(config.get("keyboard", {}).get("rotation_step", 0.05)),
    )

    server_cfg = config.get("server", {})
    host = str(args.host or server_cfg.get("host", "127.0.0.1"))
    port = int(args.port or server_cfg.get("port", 8765))
    refresh_interval_ms = int(server_cfg.get("refresh_interval_ms", 40))
    urdf_url = _to_url_path(urdf_path, project_root)

    server = make_server(
        api=api,
        keyboard=keyboard,
        static_root=project_root,
        host=host,
        port=port,
        urdf_url=urdf_url,
        refresh_interval_ms=refresh_interval_ms,
    )

    print(f"SimpleSim running at http://{host}:{port}")
    print(f"  URDF       : {urdf_url}")
    print(f"  Active arm : {active_arm_name}  ({robot.base_link} -> {robot.ee_link})")
    print(f"  Refresh    : {refresh_interval_ms} ms")
    print("Controls:")
    print("  Translate  : W/S (x)   A/D (y)   R/F (z)")
    print("  Rotate     : Q/E (yaw)   Up/Down (pitch)   Left/Right (roll)")
    if rm_bridge.error:
        print(f"RM_API2 disabled: {rm_bridge.error}")
    elif rm_bridge.enabled:
        print("RM_API2: connected")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("SimpleSim stopped.")


if __name__ == "__main__":
    main()
