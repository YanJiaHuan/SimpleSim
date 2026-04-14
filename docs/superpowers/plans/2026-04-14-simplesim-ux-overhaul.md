# SimpleSim UX Overhaul Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make SimpleSim runnable with one command and observable from the browser — no mandatory CLI args, and a web panel that shows joints, key state, pose, and IK health in real time.

**Architecture:** Keep the existing layering (`loader → core → api → interface → renderer → web`). Add one launcher script, three new backend entry points, and one rewritten frontend panel. No new Python deps. No new JS deps. Tests run under the existing pytest setup.

**Tech Stack:** Python 3 (`http.server`, `numpy`), three.js 0.166 via `/node_modules`, `third_party/urdf-loaders`, TR4 Pro URDFs, bash for the launcher.

**Spec:** `docs/superpowers/specs/2026-04-14-simplesim-ux-design.md`

---

## File Structure

**New files:**
- `run.sh` — bash launcher with submodule / node / numpy self-check, passes args through to `python main.py`.
- `dev_log.md` — project-root, reverse-chronological change log, hand-written.

**Modified files:**
- `main.py` — defaults for `--urdf` / `--config`, drop `--interface`, richer startup log, plumb arm config into `RuntimeServer`.
- `core/api.py` — `RobotAPI._q_init`, `set_robot()`, `home()`.
- `renderer/viewer.py` — store full config + URDF path, extended `/api/meta`, new `/api/home`, new `/api/switch_arm`.
- `web/index.html` — replace aside contents with six cards + keypad grid.
- `web/app.js` — render cards, keypad highlight, home / switch-arm handlers, joints table.
- `web/styles.css` — card, keypad, joint-bar styling.
- `README.md` — rewrite (philosophy + quick start + controls + layout).
- `tests/test_api.py` — add coverage for `home()` and `set_robot()`.

**Untouched (invariant):**
- `core/fk.py`, `core/ik.py`, `core/robot.py`, `core/state.py`
- `loader/urdf.py`
- `interface/keyboard.py`
- `configs/tr4.yaml`
- Anything under `third_party/`, `TR4_Pro/`, `node_modules/`

---

## Task 1: Launcher and default args

**Files:**
- Create: `run.sh`
- Modify: `main.py` (argparse defaults, drop `--interface`, startup log)

- [ ] **Step 1: Create `run.sh` with self-checks and arg passthrough**

Create `run.sh` at the project root:

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

die() { echo "error: $*" >&2; exit 1; }

[[ -f third_party/urdf-loaders/javascript/src/URDFLoader.js ]] \
  || die "missing submodule — run: git submodule update --init --recursive"

[[ -f node_modules/three/build/three.module.js ]] \
  || die "missing three.js — run: npm install"

python -c "import numpy" 2>/dev/null \
  || die "missing python deps — run: pip install numpy pytest"

exec python main.py "$@"
```

- [ ] **Step 2: Make it executable**

Run: `chmod +x run.sh`
Expected: no output.

- [ ] **Step 3: Smoke-check the script without running main.py**

Run: `bash -n run.sh`
Expected: no output (syntax OK).

- [ ] **Step 4: Modify `main.py` argparse defaults**

In `main.py`, replace `_parse_args`:

```python
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
```

Also delete the `if args.interface != "keyboard"` block. The `--interface` flag is gone; the keyboard interface is always constructed.

- [ ] **Step 5: Expand the startup log in `main.py`**

Replace the three `print` calls at the bottom of `main()` with:

```python
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
```

- [ ] **Step 6: Verify the launcher runs end-to-end (manual)**

Run: `./run.sh --port 8765`
Expected: Server prints the new banner, serves on 8765, Ctrl+C stops cleanly. If `python main.py` used to work with explicit args, this must now work without them.

Kill the process afterwards.

- [ ] **Step 7: Verify existing tests still pass**

Run: `pytest tests/ -q`
Expected: all green.

- [ ] **Step 8: Commit**

```bash
git add run.sh main.py
git commit -m "feat: run.sh launcher and default main.py args"
```

---

## Task 2: `RobotAPI.home()` and `set_robot()`

**Files:**
- Modify: `core/api.py`
- Test: `tests/test_api.py`

This task adds two methods to `RobotAPI` that power `/api/home` and `/api/switch_arm`. TDD: add the tests first, watch them fail, implement.

- [ ] **Step 1: Add failing test for `home()`**

Append to `tests/test_api.py`:

```python
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
```

- [ ] **Step 2: Add failing test for `set_robot()`**

Append to `tests/test_api.py`:

```python
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
    assert api.state.q == [0.0, 0.0]
    out = api.home()
    assert out["q"] == [0.0, 0.0]
```

- [ ] **Step 3: Run the new tests to confirm they fail**

Run: `pytest tests/test_api.py::test_home_returns_to_q_init tests/test_api.py::test_set_robot_swaps_model_and_q_init -v`
Expected: both FAIL — `TypeError: RobotAPI.__init__() got an unexpected keyword argument 'q_init'`.

- [ ] **Step 4: Update `RobotAPI.__init__` to accept and store `q_init`**

In `core/api.py`, replace the `RobotAPI.__init__` signature and body with:

```python
    def __init__(
        self,
        robot: RobotModel,
        state: SimulationState,
        ik_config: Dict[str, Any] | None = None,
        rm_bridge: RMArmBridge | None = None,
        q_init: List[float] | None = None,
    ) -> None:
        self.robot = robot
        self.state = state
        self.ik_config = ik_config or {}
        self.rm_bridge = rm_bridge
        self._lock = threading.RLock()
        self._q_init: List[float] = (
            list(q_init) if q_init is not None else list(state.q)
        )
```

The default (`list(state.q)`) keeps existing callers working — if nothing is passed, `home()` returns to wherever the robot started.

- [ ] **Step 5: Add `home()` and `set_robot()` to `RobotAPI`**

In `core/api.py`, add these methods after `get_ee_pose`:

```python
    def home(self) -> Dict[str, Any]:
        with self._lock:
            return self.move_joint(self._q_init[:])

    def set_robot(
        self,
        robot: RobotModel,
        state: SimulationState,
        q_init: List[float],
    ) -> Dict[str, Any]:
        with self._lock:
            self.robot = robot
            self.state = state
            self._q_init = list(q_init)
            return self._snapshot_locked()
```

`home()` reuses `move_joint`, which already handles FK refresh and the optional RM bridge. `set_robot` is atomic under the lock so a switch can't race with a polling read.

- [ ] **Step 6: Run the new tests to confirm they pass**

Run: `pytest tests/test_api.py -v`
Expected: all tests in the file PASS (existing two + new two).

- [ ] **Step 7: Run the full suite to confirm no regressions**

Run: `pytest tests/ -q`
Expected: all green.

- [ ] **Step 8: Update `main.py` to pass `q_init` into `RobotAPI`**

In `main.py`, change the `RobotAPI(...)` construction to:

```python
    q_init = [float(v) for v in arm_config.get("q_init", [0.0] * robot.dof)]
    state = make_initial_state(robot, q_init)
    rm_bridge = RMArmBridge(config.get("rm_api", {}))
    api = RobotAPI(
        robot=robot,
        state=state,
        ik_config=config.get("ik", {}),
        rm_bridge=rm_bridge,
        q_init=q_init,
    )
```

Delete the old duplicate `q_init` inlining if it existed.

- [ ] **Step 9: Commit**

```bash
git add core/api.py tests/test_api.py main.py
git commit -m "feat(core): RobotAPI.home and set_robot with q_init"
```

---

## Task 3: Backend `/api/home`, `/api/switch_arm`, extended `/api/meta`

**Files:**
- Modify: `renderer/viewer.py`
- Modify: `main.py`

This task wires the new `RobotAPI` methods into HTTP endpoints and carries enough context in `RuntimeServer` to rebuild a `RobotModel` on demand.

- [ ] **Step 1: Extend `RuntimeServer.__init__` with config + arm context**

In `renderer/viewer.py`, replace `RuntimeServer.__init__` with:

```python
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
```

Also update `make_server(...)` at the bottom of the file to accept and forward the same new parameters.

- [ ] **Step 2: Add a helper that builds meta payloads**

In `renderer/viewer.py`, inside `RuntimeServer`, add:

```python
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
```

- [ ] **Step 3: Use `_meta_payload()` in `/api/meta` handler**

Still in `renderer/viewer.py`, inside `_build_handler`, change the `/api/meta` GET branch to reference the server's `_meta_payload()`. The handler class currently closes over locals; to access the server, close over `server_ref = self` before defining `Handler` and call `server_ref._meta_payload()`.

Full replacement of `_build_handler`:

```python
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
```

- [ ] **Step 4: Add `_switch_arm` on `RuntimeServer`**

In `renderer/viewer.py`, add this method on `RuntimeServer` (just below `_meta_payload`):

```python
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
```

Add the new imports at the top of `renderer/viewer.py`:

```python
from core.state import make_initial_state
from loader.urdf import load_robot_model
```

- [ ] **Step 5: Plumb the new parameters from `main.py`**

In `main.py`, replace the `make_server(...)` call with:

```python
    server = make_server(
        api=api,
        keyboard=keyboard,
        static_root=project_root,
        host=host,
        port=port,
        urdf_url=urdf_url,
        urdf_path=urdf_path,
        config=config,
        active_arm=active_arm_name,
        refresh_interval_ms=refresh_interval_ms,
    )
```

Update the `make_server` wrapper at the bottom of `renderer/viewer.py` to match:

```python
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
```

- [ ] **Step 6: Smoke-test the new endpoints (manual)**

Start: `./run.sh --port 8765` in one shell.
In another shell:

```bash
curl -s http://127.0.0.1:8765/api/meta | python -m json.tool
curl -s -X POST http://127.0.0.1:8765/api/home | python -m json.tool
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"arm":"right"}' http://127.0.0.1:8765/api/switch_arm | python -m json.tool
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"arm":"left"}' http://127.0.0.1:8765/api/switch_arm | python -m json.tool
```

Expected:
- `/api/meta` returns `joint_limits`, `active_arm`, `available_arms`, `q_init`, `joint_names`.
- `/api/home` returns a state with `q` equal to the `q_init` from meta.
- `/api/switch_arm` with `"right"` flips `active_arm` to `right`, and meta's `base_link`/`ee_link` change accordingly.
- Switching back to `"left"` restores the original.

Kill the server afterwards.

- [ ] **Step 7: Run full tests**

Run: `pytest tests/ -q`
Expected: all green.

- [ ] **Step 8: Commit**

```bash
git add renderer/viewer.py main.py
git commit -m "feat(api): /api/home, /api/switch_arm, extended meta"
```

---

## Task 4: Frontend HTML + CSS cards

**Files:**
- Modify: `web/index.html`
- Modify: `web/styles.css`

This task only lays out structure and styles. Behavior comes in Task 5.

- [ ] **Step 1: Replace the aside in `web/index.html`**

Replace the `<aside class="panel">...</aside>` block with:

```html
  <aside class="panel">
    <header class="panel-header">
      <h1>SimpleSim</h1>
      <p id="status" class="status">booting</p>
    </header>

    <section class="card" id="card-info">
      <h2>Robot</h2>
      <p id="robot-name" class="muted">loading...</p>
      <p id="robot-chain" class="muted"></p>
      <p>Arm: <span id="arm-badge" class="badge">-</span></p>
    </section>

    <section class="card" id="card-keypad">
      <h2>Keypad</h2>
      <div class="keypad">
        <div class="keypad-row">
          <div class="key" data-code="KeyQ">Q</div>
          <div class="key" data-code="KeyW">W</div>
          <div class="key" data-code="KeyE">E</div>
          <div class="key" data-code="KeyR">R</div>
        </div>
        <div class="keypad-row">
          <div class="key" data-code="KeyA">A</div>
          <div class="key" data-code="KeyS">S</div>
          <div class="key" data-code="KeyD">D</div>
          <div class="key" data-code="KeyF">F</div>
        </div>
        <div class="keypad-row">
          <div class="key" data-code="ArrowUp">&uarr;</div>
          <div class="key" data-code="ArrowDown">&darr;</div>
          <div class="key" data-code="ArrowLeft">&larr;</div>
          <div class="key" data-code="ArrowRight">&rarr;</div>
        </div>
      </div>
      <p class="hint">WASD/RF translate · QE yaw · arrows pitch/roll</p>
    </section>

    <section class="card" id="card-pose">
      <h2>EE Pose</h2>
      <dl class="pose">
        <dt>x</dt><dd id="pose-x">-</dd>
        <dt>y</dt><dd id="pose-y">-</dd>
        <dt>z</dt><dd id="pose-z">-</dd>
        <dt>roll</dt><dd id="pose-roll">-</dd>
        <dt>pitch</dt><dd id="pose-pitch">-</dd>
        <dt>yaw</dt><dd id="pose-yaw">-</dd>
      </dl>
    </section>

    <section class="card" id="card-joints">
      <h2>Joints</h2>
      <table class="joints">
        <thead>
          <tr><th>name</th><th>rad</th><th>deg</th><th></th></tr>
        </thead>
        <tbody id="joints-body"></tbody>
      </table>
    </section>

    <section class="card" id="card-ik">
      <h2>IK</h2>
      <p><span id="ik-dot" class="dot"></span> <span id="ik-summary">idle</span></p>
    </section>

    <section class="card" id="card-actions">
      <h2>Actions</h2>
      <div class="actions">
        <button type="button" id="btn-home">Home</button>
        <button type="button" id="btn-arm">Switch Arm</button>
      </div>
    </section>
  </aside>
```

- [ ] **Step 2: Replace `web/styles.css` with the card styling**

Overwrite `web/styles.css`:

```css
:root {
  --bg: #0f172a;
  --panel: #111827;
  --card: #1f2937;
  --border: #334155;
  --text: #e2e8f0;
  --muted: #94a3b8;
  --accent: #22d3ee;
  --good: #22c55e;
  --bad: #ef4444;
}

* { box-sizing: border-box; }

html, body {
  margin: 0;
  height: 100%;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  color: var(--text);
  background: var(--bg);
}

body {
  display: flex;
}

.panel {
  width: 340px;
  padding: 16px;
  background: var(--panel);
  border-right: 1px solid var(--border);
  overflow-y: auto;
}

.panel-header h1 {
  margin: 0 0 4px 0;
  font-size: 18px;
  letter-spacing: 0.04em;
}

.status {
  margin: 0 0 12px 0;
  font-size: 12px;
  color: var(--muted);
}

.card {
  background: var(--card);
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 10px 12px;
  margin-bottom: 10px;
}

.card h2 {
  margin: 0 0 8px 0;
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  color: var(--muted);
}

.muted { color: var(--muted); font-size: 12px; margin: 2px 0; }

.badge {
  display: inline-block;
  padding: 2px 8px;
  border-radius: 10px;
  background: var(--accent);
  color: #0f172a;
  font-weight: 600;
  font-size: 11px;
  letter-spacing: 0.05em;
}

.keypad { display: flex; flex-direction: column; gap: 4px; }
.keypad-row { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px; }
.key {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 12px;
  padding: 8px 0;
  text-align: center;
  background: #0f172a;
  border: 1px solid var(--border);
  border-radius: 4px;
  user-select: none;
  transition: background 40ms, color 40ms;
}
.key.active {
  background: var(--accent);
  color: #0f172a;
  border-color: var(--accent);
}
.hint {
  margin: 8px 0 0 0;
  font-size: 11px;
  color: var(--muted);
}

.pose {
  display: grid;
  grid-template-columns: auto 1fr;
  column-gap: 10px;
  row-gap: 2px;
  margin: 0;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 12px;
}
.pose dt { color: var(--muted); }
.pose dd { margin: 0; text-align: right; }

.joints { width: 100%; font-size: 11px; border-collapse: collapse; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
.joints th { text-align: left; color: var(--muted); font-weight: 500; padding-bottom: 4px; }
.joints td { padding: 2px 4px 2px 0; }
.joints td.num { text-align: right; }
.joint-bar { position: relative; height: 4px; background: #0f172a; border-radius: 2px; overflow: hidden; }
.joint-bar > span { position: absolute; top: 0; bottom: 0; background: var(--accent); }

.dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: var(--muted); margin-right: 6px; vertical-align: middle; }
.dot.good { background: var(--good); }
.dot.bad { background: var(--bad); }

.actions { display: flex; gap: 6px; }
.actions button {
  flex: 1;
  padding: 6px 0;
  background: #0f172a;
  color: var(--text);
  border: 1px solid var(--border);
  border-radius: 4px;
  cursor: pointer;
  font-size: 12px;
}
.actions button:hover { border-color: var(--accent); }

#viewer { flex: 1; position: relative; }
#viewer canvas { display: block; }
```

- [ ] **Step 3: Visual sanity check (manual)**

Start: `./run.sh`
Open `http://127.0.0.1:8765`.
Expected: Six cards render in the left panel. The three.js viewer still shows the robot. Joints table shows an empty body (Task 5 fills it). Pose dd cells show `-`. Keypad cells are static.

Kill the server.

- [ ] **Step 4: Commit**

```bash
git add web/index.html web/styles.css
git commit -m "feat(web): 6-card panel layout and styling"
```

---

## Task 5: Frontend behavior

**Files:**
- Modify: `web/app.js`

This rewires `app.js` to populate every card, highlight keys, and handle Home / Switch Arm.

- [ ] **Step 1: Replace `web/app.js` top section (imports + DOM refs + state)**

Overwrite the file head (lines 1-45 in the current file) with:

```javascript
import {
  AmbientLight,
  Box3,
  Color,
  DirectionalLight,
  Mesh,
  PerspectiveCamera,
  PlaneGeometry,
  Scene,
  ShadowMaterial,
  Vector3,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from '/third_party/urdf-loaders/javascript/src/URDFLoader.js';

const viewer = document.getElementById('viewer');
const statusEl = document.getElementById('status');
const robotNameEl = document.getElementById('robot-name');
const robotChainEl = document.getElementById('robot-chain');
const armBadgeEl = document.getElementById('arm-badge');
const jointsBodyEl = document.getElementById('joints-body');
const ikDotEl = document.getElementById('ik-dot');
const ikSummaryEl = document.getElementById('ik-summary');
const homeBtn = document.getElementById('btn-home');
const armBtn = document.getElementById('btn-arm');

const poseEls = {
  x: document.getElementById('pose-x'),
  y: document.getElementById('pose-y'),
  z: document.getElementById('pose-z'),
  roll: document.getElementById('pose-roll'),
  pitch: document.getElementById('pose-pitch'),
  yaw: document.getElementById('pose-yaw'),
};

const keyEls = new Map();
document.querySelectorAll('.key').forEach(el => {
  keyEls.set(el.dataset.code, el);
});

const activeKeys = new Set();
const acceptedKeys = new Set([
  'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyR', 'KeyF',
  'KeyQ', 'KeyE', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight',
]);

let meta = null;
let robot = null;
let robotRoot = null;
let renderer = null;
let scene = null;
let camera = null;
let controls = null;
let busy = false;
```

- [ ] **Step 2: Replace `formatPose` / `applyState` with card-aware versions**

Replace the existing `formatPose` and `applyState` functions with:

```javascript
const RAD2DEG = 180 / Math.PI;

function fmt(value, digits = 4) {
  return Number(value).toFixed(digits);
}

function applyPose(pose) {
  if (!pose || pose.length !== 6) return;
  poseEls.x.textContent = fmt(pose[0]);
  poseEls.y.textContent = fmt(pose[1]);
  poseEls.z.textContent = fmt(pose[2]);
  poseEls.roll.textContent = fmt(pose[3] * RAD2DEG, 2);
  poseEls.pitch.textContent = fmt(pose[4] * RAD2DEG, 2);
  poseEls.yaw.textContent = fmt(pose[5] * RAD2DEG, 2);
}

function applyJoints(jointNames, q) {
  if (!jointNames || !q || !meta) return;

  // Build rows once per meta change.
  if (jointsBodyEl.childElementCount !== jointNames.length) {
    jointsBodyEl.innerHTML = '';
    for (let i = 0; i < jointNames.length; i += 1) {
      const row = document.createElement('tr');
      row.innerHTML = `
        <td>${jointNames[i]}</td>
        <td class="num" data-role="rad">-</td>
        <td class="num" data-role="deg">-</td>
        <td><div class="joint-bar"><span></span></div></td>
      `;
      jointsBodyEl.appendChild(row);
    }
  }

  const rows = jointsBodyEl.children;
  for (let i = 0; i < jointNames.length; i += 1) {
    const row = rows[i];
    if (!row) continue;
    const limits = meta.joint_limits?.[jointNames[i]] || [-Math.PI, Math.PI];
    const [lower, upper] = limits;
    const span = Math.max(upper - lower, 1e-6);
    const frac = Math.min(1, Math.max(0, (q[i] - lower) / span));

    row.querySelector('[data-role="rad"]').textContent = fmt(q[i], 4);
    row.querySelector('[data-role="deg"]').textContent = fmt(q[i] * RAD2DEG, 2);
    const bar = row.querySelector('.joint-bar > span');
    bar.style.left = '0%';
    bar.style.width = `${(frac * 100).toFixed(1)}%`;
  }
}

function applyIk(ik, success) {
  if (ik) {
    ikDotEl.classList.remove('good', 'bad');
    ikDotEl.classList.add(success === false ? 'bad' : 'good');
    ikSummaryEl.textContent =
      `iter=${ik.iterations} pos=${fmt(ik.position_error)} rot=${fmt(ik.orientation_error)}`;
  } else if (success === false) {
    ikDotEl.classList.remove('good');
    ikDotEl.classList.add('bad');
    ikSummaryEl.textContent = 'failed';
  }
}

function applyState(state) {
  if (!state) return;

  if (state.ee_pose) applyPose(state.ee_pose);
  if (state.joint_names && state.q) applyJoints(state.joint_names, state.q);
  if (state.ik || state.success === false) applyIk(state.ik, state.success);

  if (robot && state.joint_names && state.q) {
    const joints = {};
    for (let i = 0; i < state.joint_names.length; i += 1) {
      joints[state.joint_names[i]] = state.q[i];
    }
    robot.setJointValues(joints);
    robot.updateMatrixWorld(true);
  }
}
```

- [ ] **Step 3: Extend `pollState` / `sendKeyboard` (already present) and add Home / Switch handlers**

Leave `fetchJson`, `pollState`, `sendKeyboard`, `initScene`, `animate` as they are — but replace `sendKeyboard`'s status message with a shorter one:

```javascript
async function sendKeyboard() {
  if (busy || activeKeys.size === 0) return;
  busy = true;
  try {
    const payload = { keys: Array.from(activeKeys) };
    const state = await fetchJson('/api/keyboard', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    statusEl.textContent = state.success ? 'running' : 'ik failed';
    applyState(state);
  } catch (err) {
    statusEl.textContent = `keyboard failed: ${err.message}`;
  } finally {
    busy = false;
  }
}

async function doHome() {
  try {
    const state = await fetchJson('/api/home', { method: 'POST' });
    statusEl.textContent = 'home';
    applyState(state);
  } catch (err) {
    statusEl.textContent = `home failed: ${err.message}`;
  }
}

async function doSwitchArm() {
  if (!meta) return;
  const arms = meta.available_arms || [];
  if (arms.length < 2) return;
  const next = arms.find(a => a !== meta.active_arm) || arms[0];
  try {
    const response = await fetchJson('/api/switch_arm', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ arm: next }),
    });
    meta = response.meta;
    applyMeta(meta);
    await reloadRobot();
    applyState(response.state);
    statusEl.textContent = `arm: ${next}`;
  } catch (err) {
    statusEl.textContent = `switch failed: ${err.message}`;
  }
}
```

- [ ] **Step 4: Add `applyMeta` and refactor `loadRobot` into `reloadRobot`**

Replace the existing `loadRobot` with these two functions:

```javascript
function applyMeta(nextMeta) {
  robotNameEl.textContent = nextMeta.robot_name;
  robotChainEl.textContent = `${nextMeta.base_link} → ${nextMeta.ee_link}`;
  armBadgeEl.textContent = nextMeta.active_arm;
  // Reset joints table so it rebuilds on next applyState.
  jointsBodyEl.innerHTML = '';
}

async function reloadRobot() {
  if (robotRoot) {
    scene.remove(robotRoot);
    robotRoot = null;
    robot = null;
  }

  const loader = new URDFLoader();
  const loaded = await new Promise((resolve, reject) => {
    loader.load(meta.urdf_url, resolve, undefined, reject);
  });

  loaded.traverse(node => { node.castShadow = true; });
  scene.add(loaded);
  loaded.updateMatrixWorld(true);

  robot = loaded;
  robotRoot = loaded;

  const box = new Box3().setFromObject(loaded);
  const center = new Vector3();
  box.getCenter(center);
  controls.target.copy(center);
  camera.position.set(center.x + 2.0, center.y + 1.4, center.z + 2.0);
  controls.update();
}
```

- [ ] **Step 5: Wire keypad highlight and button handlers**

Replace `setupKeyboard` with a version that also toggles CSS and bind the two buttons:

```javascript
function setupKeyboard() {
  document.addEventListener('keydown', event => {
    if (!acceptedKeys.has(event.code)) return;
    event.preventDefault();
    activeKeys.add(event.code);
    keyEls.get(event.code)?.classList.add('active');
  });

  document.addEventListener('keyup', event => {
    if (!acceptedKeys.has(event.code)) return;
    event.preventDefault();
    activeKeys.delete(event.code);
    keyEls.get(event.code)?.classList.remove('active');
  });

  window.addEventListener('blur', () => {
    activeKeys.clear();
    keyEls.forEach(el => el.classList.remove('active'));
  });

  homeBtn.addEventListener('click', doHome);
  armBtn.addEventListener('click', doSwitchArm);
}
```

- [ ] **Step 6: Rewrite `init()` to use `applyMeta` + `reloadRobot`**

Replace the existing `init()` with:

```javascript
async function init() {
  statusEl.textContent = 'loading meta';
  meta = await fetchJson('/api/meta');
  applyMeta(meta);

  initScene();
  setupKeyboard();

  statusEl.textContent = 'loading urdf';
  await reloadRobot();

  const refreshMs = Number(meta.refresh_interval_ms || 40);
  setInterval(pollState, refreshMs);
  setInterval(sendKeyboard, refreshMs);

  await pollState();
  statusEl.textContent = 'ready';
  animate();
}

init().catch(err => {
  statusEl.textContent = `startup failed: ${err.message}`;
});
```

- [ ] **Step 7: Full manual verification**

Start: `./run.sh`
Open `http://127.0.0.1:8765` in a browser.

Expected:
1. Six cards populate: robot name + chain + arm badge (`left`), keypad in default state, EE pose with numeric values, joints table with 6 rows and bars, IK card shows `idle` or green dot after first poll.
2. Hold `W` — the `W` key highlights cyan, EE pose `x` drifts, joints table updates, IK dot stays green.
3. Release `W` — highlight clears.
4. Click `Home` — joints return to `q_init`, pose returns to starting value, status shows `home`.
5. Click `Switch Arm` — badge flips to `right`, the 3D scene re-centers on the right arm base, joints table rebuilds with right-arm joint names, pose updates.
6. Hold `A` on the right arm — it still drives. Click `Switch Arm` again to flip back to `left`.
7. `window.blur` (Alt-Tab) — any highlighted keys clear.

Kill the server.

- [ ] **Step 8: Full test run**

Run: `pytest tests/ -q`
Expected: all green.

- [ ] **Step 9: Commit**

```bash
git add web/app.js
git commit -m "feat(web): live joints, keypad highlight, home, switch arm"
```

---

## Task 6: README and dev_log

**Files:**
- Modify: `README.md`
- Create: `dev_log.md`

- [ ] **Step 1: Overwrite `README.md`**

Replace the two-line `README.md` with:

```markdown
# SimpleSim

A minimal browser-based runtime for URDF-driven robot control verification.
Current simulators are heavy; URDF viewers can only look. SimpleSim loads a
URDF, renders it in the browser, drives it with keyboard → IK → FK, and
optionally bridges to a real arm via RM_API2. No ROS. No physics. Just enough
to verify control logic end-to-end.

## Quick start

```bash
git submodule update --init --recursive
npm install
./run.sh
```

Then open <http://127.0.0.1:8765>.

`run.sh` checks submodules, `node_modules/three`, and `numpy` before it starts
the server. It never installs anything silently — it tells you the command to
run if something is missing.

## Controls

| Action | Keys |
|---|---|
| Translate X | `W` / `S` |
| Translate Y | `A` / `D` |
| Translate Z | `R` / `F` |
| Yaw         | `Q` / `E` |
| Pitch       | `↑` / `↓` |
| Roll        | `←` / `→` |

The left panel also shows live joint angles, the current EE pose, IK health,
and buttons for `Home` and `Switch Arm` (left/right on the TR4 Pro).

## Layout

```
core/       # RobotModel, FK, DLS IK, state, RobotAPI (only controller entry)
loader/     # URDF → RobotModel
interface/  # keyboard → EE delta command
renderer/   # http.server + /api endpoints + static files
web/        # three.js + URDFLoader frontend
configs/    # JSON-formatted YAML (active arm, IK, server, keyboard, rm_api)
```

Layer rule: `interface` and `renderer` never touch `state` directly — all
mutation goes through `core/api.py`.

## Tests

```bash
pytest tests/
```
```

- [ ] **Step 2: Create `dev_log.md`**

Write to `dev_log.md`:

```markdown
# SimpleSim Dev Log

Reverse-chronological. Each entry: date — subject, bullet list of concrete
changes, short motivation. Hand-written on every change. No git hooks.

## 2026-04-14 — UX overhaul

- `run.sh` launcher with submodule / three / numpy self-check; `main.py`
  gets default `--urdf` / `--config`, drops the dead `--interface` flag.
- `RobotAPI` gains `_q_init`, `home()`, `set_robot()`; `/api/home` and
  `/api/switch_arm` wired in `renderer/viewer.py`; `/api/meta` now exposes
  `joint_limits`, `active_arm`, `available_arms`, `q_init`.
- Web panel rebuilt as six cards (info / keypad / pose / joints / ik /
  actions). Keypad highlights on keydown. `Home` resets to `q_init`.
  `Switch Arm` re-loads the right/left arm with the same URDF.
- `README.md` rewritten (philosophy + quick start + controls + layout).
- Motivation: author forgot how to start the project; web panel showed
  only raw EE pose. Closing the easy-to-use gap before adding features.
```

- [ ] **Step 3: Final manual verification of the happy path**

Start: `./run.sh`
Expected:
- Banner shows URDF, arm, refresh, controls block.
- Browser loads; all six cards render; keyboard drives EE; Home and Switch Arm both work.
- Ctrl+C exits cleanly.

- [ ] **Step 4: Final test run**

Run: `pytest tests/ -q`
Expected: all green.

- [ ] **Step 5: Commit**

```bash
git add README.md dev_log.md
git commit -m "docs: rewrite README and start dev_log"
```

---

## Post-implementation checklist

- [ ] `./run.sh` starts with zero args on a clean shell
- [ ] `pytest tests/` green
- [ ] Six cards render correctly
- [ ] `Home` and `Switch Arm` buttons function
- [ ] `README.md` is the only narrative doc at the root besides `dev_log.md` and `design.md`
- [ ] No new `.md` files outside `docs/superpowers/` + `README.md` + `dev_log.md`
- [ ] `dev_log.md` entry dated `2026-04-14` exists
