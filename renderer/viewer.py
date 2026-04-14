from __future__ import annotations

import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict
from urllib.parse import urlparse

from core.api import RobotAPI
from core.state import make_initial_state
from interface.keyboard import KeyboardInterface
from loader.urdf import load_robot_model


def _json_body(handler: SimpleHTTPRequestHandler) -> Dict[str, Any]:
    content_length = int(handler.headers.get("Content-Length", "0"))
    if content_length <= 0:
        return {}

    raw = handler.rfile.read(content_length)
    if not raw:
        return {}
    return json.loads(raw.decode("utf-8"))


def _send_json(handler: SimpleHTTPRequestHandler, payload: Dict[str, Any], status: int = 200) -> None:
    blob = json.dumps(payload).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(blob)))
    handler.end_headers()
    handler.wfile.write(blob)


class RuntimeServer:
    def __init__(
        self,
        api: RobotAPI,
        keyboard: KeyboardInterface,
        static_root: Path,
        host: str,
        port: int,
        urdf_url: str,
        urdf_path: Path,
        config: Dict[str, Any],
        active_arm: str,
        refresh_interval_ms: int = 40,
    ) -> None:
        self.api = api
        self.keyboard = keyboard
        self.static_root = static_root
        self.host = host
        self.port = port
        self.urdf_url = urdf_url
        self.urdf_path = urdf_path
        self.config = config
        self.active_arm = active_arm
        self.refresh_interval_ms = refresh_interval_ms

    def _meta_payload(self) -> Dict[str, Any]:
        robot = self.api.robot
        joint_limits = {
            name: [limit.lower, limit.upper]
            for name, limit in robot.joint_limits.items()
        }
        arm_cfg = self.config.get("arms", {}).get(self.active_arm, {})
        q_init = [float(v) for v in arm_cfg.get("q_init", [0.0] * robot.dof)]
        return {
            "urdf_url": self.urdf_url,
            "robot_name": robot.name,
            "base_link": robot.base_link,
            "ee_link": robot.ee_link,
            "joint_names": robot.joint_names,
            "joint_limits": joint_limits,
            "q_init": q_init,
            "active_arm": self.active_arm,
            "available_arms": sorted(self.config.get("arms", {}).keys()),
            "refresh_interval_ms": self.refresh_interval_ms,
        }

    def _switch_arm(self, arm_name: str) -> Dict[str, Any]:
        arms = self.config.get("arms", {})
        if arm_name not in arms:
            raise ValueError(f"unknown arm '{arm_name}'")

        arm_cfg = arms[arm_name]
        robot = load_robot_model(
            urdf_path=str(self.urdf_path),
            base_link=str(arm_cfg["base_link"]),
            ee_link=str(arm_cfg["ee_link"]),
            expected_joint_names=[str(n) for n in arm_cfg.get("joint_names", [])],
        )
        q_init = [float(v) for v in arm_cfg.get("q_init", [0.0] * robot.dof)]
        state = make_initial_state(robot, q_init)
        self.api.set_robot(robot=robot, state=state, q_init=q_init)
        self.active_arm = arm_name
        return self.api.snapshot()

    def _build_handler(self):
        server_ref = self
        directory = str(self.static_root)

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=directory, **kwargs)

            def log_message(self, format: str, *args) -> None:
                return

            def do_GET(self):
                path = urlparse(self.path).path

                if path == "/api/state":
                    return _send_json(self, server_ref.api.snapshot())

                if path == "/api/meta":
                    return _send_json(self, server_ref._meta_payload())

                if path == "/":
                    self.path = "/web/index.html"
                    return super().do_GET()

                return super().do_GET()

            def do_POST(self):
                path = urlparse(self.path).path

                try:
                    payload = _json_body(self)
                except json.JSONDecodeError:
                    return _send_json(self, {"success": False, "error": "invalid json"}, status=400)

                try:
                    if path == "/api/keyboard":
                        keys = payload.get("keys", [])
                        return _send_json(
                            self,
                            server_ref.keyboard.apply_keys(keys, server_ref.api),
                        )

                    if path == "/api/move_joint":
                        return _send_json(self, server_ref.api.move_joint(payload["q"]))

                    if path == "/api/move_ee":
                        return _send_json(self, server_ref.api.move_ee(payload["pose"]))

                    if path == "/api/home":
                        return _send_json(self, server_ref.api.home())

                    if path == "/api/switch_arm":
                        arm_name = str(payload.get("arm", ""))
                        snapshot = server_ref._switch_arm(arm_name)
                        return _send_json(
                            self,
                            {"meta": server_ref._meta_payload(), "state": snapshot},
                        )

                    return _send_json(self, {"success": False, "error": "not found"}, status=404)
                except Exception as exc:
                    return _send_json(self, {"success": False, "error": str(exc)}, status=400)

        return Handler

    def serve_forever(self) -> None:
        handler_cls = self._build_handler()
        server = ThreadingHTTPServer((self.host, self.port), handler_cls)
        try:
            server.serve_forever()
        finally:
            server.server_close()


def make_server(
    api: RobotAPI,
    keyboard: KeyboardInterface,
    static_root: Path,
    host: str,
    port: int,
    urdf_url: str,
    urdf_path: Path,
    config: Dict[str, Any],
    active_arm: str,
    refresh_interval_ms: int = 40,
) -> RuntimeServer:
    return RuntimeServer(
        api=api,
        keyboard=keyboard,
        static_root=static_root,
        host=host,
        port=port,
        urdf_url=urdf_url,
        urdf_path=urdf_path,
        config=config,
        active_arm=active_arm,
        refresh_interval_ms=refresh_interval_ms,
    )
