from __future__ import annotations

import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict
from urllib.parse import urlparse

from core.api import RobotAPI
from interface.keyboard import KeyboardInterface


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
        refresh_interval_ms: int = 40,
    ) -> None:
        self.api = api
        self.keyboard = keyboard
        self.static_root = static_root
        self.host = host
        self.port = port
        self.urdf_url = urdf_url
        self.refresh_interval_ms = refresh_interval_ms

    def _build_handler(self):
        api = self.api
        keyboard = self.keyboard
        urdf_url = self.urdf_url
        refresh_interval_ms = self.refresh_interval_ms
        directory = str(self.static_root)

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=directory, **kwargs)

            def log_message(self, format: str, *args) -> None:
                return

            def do_GET(self):
                path = urlparse(self.path).path

                if path == "/api/state":
                    return _send_json(self, api.snapshot())

                if path == "/api/meta":
                    return _send_json(
                        self,
                        {
                            "urdf_url": urdf_url,
                            "joint_names": api.robot.joint_names,
                            "base_link": api.robot.base_link,
                            "ee_link": api.robot.ee_link,
                            "refresh_interval_ms": refresh_interval_ms,
                            "robot_name": api.robot.name,
                        },
                    )

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
                        return _send_json(self, keyboard.apply_keys(keys, api))

                    if path == "/api/move_joint":
                        return _send_json(self, api.move_joint(payload["q"]))

                    if path == "/api/move_ee":
                        return _send_json(self, api.move_ee(payload["pose"]))

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
    refresh_interval_ms: int = 40,
) -> RuntimeServer:
    return RuntimeServer(
        api=api,
        keyboard=keyboard,
        static_root=static_root,
        host=host,
        port=port,
        urdf_url=urdf_url,
        refresh_interval_ms=refresh_interval_ms,
    )
