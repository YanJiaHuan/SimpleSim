# SimpleSim UX Overhaul — Design Spec

Date: 2026-04-14
Scope: startup ergonomics, Web UI observability, docs.
Out of scope: multi-robot config, trajectory visualization, joint sliders, physics,
new IK algorithms, hot-reload, automated dev-log generation.

## Motivation

SimpleSim is a minimal browser-based runtime for URDF-driven robot control
verification. The prototype works (FK, DLS IK, TR4 Pro, keyboard → EE delta, optional
RM_API2 hardware bridge), but two usability gaps block the author from returning to
the project after a break:

1. No quick-start. `README.md` is empty; `main.py` requires two absolute-ish
   arguments; the author forgot the launch incantation.
2. The Web UI shows only a robot name, a status string, raw EE pose array, and an IK
   JSON blob. Nothing about joints, no key feedback, no reset, no arm switching.

This overhaul closes both gaps without expanding scope.

## Non-Goals

- Supporting robots other than TR4 Pro in this iteration.
- Changing FK/IK math.
- Replacing `http.server` with a web framework.
- Auto-generating the dev log from git.
- Persisting UI preferences across sessions.

## A. Startup Experience

### `run.sh` (new, project root, executable)

Bash script that performs four steps and exits non-zero with a helpful message if any
check fails:

1. **Submodule check** — verify `third_party/urdf-loaders/javascript/src/URDFLoader.js`
   exists. On miss, print: `missing submodule — run: git submodule update --init --recursive`.
2. **Node modules check** — verify `node_modules/three/build/three.module.js`
   exists. On miss, print: `missing three.js — run: npm install`.
3. **Python deps check** — `python -c "import numpy"`. On miss, print:
   `missing python deps — run: pip install numpy pytest`.
4. **Run** — `exec python main.py "$@"` so user args pass through and signals
   propagate cleanly.

Rationale: A2 variant of the launcher spectrum. Checks but does not auto-install,
so running the script never mutates the environment silently.

### `main.py` changes

- `--urdf` default: `TR4_Pro/TR4_with_grippers_v2.urdf` (was required).
- `--config` default: `configs/tr4.yaml` (was required).
- Remove `--interface` argument entirely. It had `choices=["keyboard"]` with a single
  option and was dead code — the `KeyboardInterface` is always constructed.
- Startup log expands from one control line to: address, URDF url, active arm name,
  full keymap (WASD translate / RF up-down / QE yaw / arrows pitch-roll), and the
  RM_API2 bridge status.

Result: `./run.sh` from project root is the fully happy path. Zero mandatory args.

## B. Web UI Observability

### Layout

`web/index.html` panel is replaced by six cards, stacked vertically in the existing
left aside, with the three.js viewer unchanged on the right.

| Card | Content |
|---|---|
| Robot Info | robot name, `base_link → ee_link`, active arm badge (`left` / `right`) |
| Keypad (B2) | 12-key grid: WASD, R/F, Q/E, arrows. CSS class `active` toggled on keydown/keyup. Display only — no click handlers. |
| EE Pose (B3) | Two labeled rows: `x / y / z` in meters, `roll / pitch / yaw` in degrees. Fixed 4-decimal formatting, monospace. |
| Joints (B1) | Table: joint name, rad (4 dp), deg (2 dp), a per-row bar showing `(q - lower) / (upper - lower)` with `lower` / `upper` markers. |
| IK Status (B6) | Colored dot (green `success=true`, red otherwise), plus `iter / pos_err / rot_err`. |
| Actions | `Home` button → `POST /api/home`. `Arm: left | right` toggle → `POST /api/switch_arm`. |

### Data flow

- Existing `setInterval(pollState)` at `refresh_interval_ms` (default 40 ms) remains
  the single source of joint / pose / IK updates. `applyState` is extended to fan
  out into the new cards.
- Keypad highlighting is pure frontend: `keydown` adds CSS class `active` to the
  matching `.key[data-code="KeyW"]`, `keyup` removes it. No extra network.
- `Home` and `Arm` buttons wait on their POST responses and then call `applyState`
  with the returned snapshot.

### Backend API changes (`renderer/viewer.py`)

New endpoints:

- `POST /api/home` — body empty. Calls `api.home()` (see below). Returns a state
  snapshot identical in shape to `/api/state`.
- `POST /api/switch_arm` — body `{"arm": "left" | "right"}`. Validates the name
  against `RuntimeServer.arms`. On success, re-loads the `RobotModel` from the
  stored URDF path using the new arm config, replaces `api.robot` / `api.state`,
  returns `{meta, snapshot}` so the frontend can update both meta and state in one
  round trip.

Extended `GET /api/meta` payload adds:

- `active_arm: "left" | "right"`
- `available_arms: ["left", "right"]`
- `joint_limits: {joint_name: [lower, upper]}` (derived from `robot.joint_limits`)
- `q_init: [float]` (for client-side Home preview and future use)

Existing fields (`urdf_url`, `joint_names`, `base_link`, `ee_link`,
`refresh_interval_ms`, `robot_name`) remain.

### Backend core changes

- `RobotAPI` gains private field `_q_init: list[float]`, set by the constructor and
  by a new `set_robot(robot, state, q_init)` method. `set_robot` acquires the
  existing `_lock`, replaces `self.robot` / `self.state` / `self._q_init`
  atomically, and is the only way `switch_arm` mutates the API.
- `RobotAPI.home()` — new method. Under `_lock`, calls `self.move_joint(self._q_init[:])`
  and returns its snapshot. This reuses the FK + RM-bridge path already in
  `move_joint`, so no duplicated logic.
- `RuntimeServer` stores the project root, URDF path, parsed config dict, and
  current arm name so `/api/switch_arm` can rebuild `RobotModel` / `SimulationState`
  and hand them to `api.set_robot(...)`. `main.py` passes these in.

### State invariants

- `UI is forbidden to mutate joint state directly` (from `design.md`) remains true.
  All new UI actions go through existing `api.move_joint` / `api.step_ee` paths.
- `switch_arm` resets the target arm to its own `q_init`. The previous arm's joint
  state is discarded. This keeps the controller stateless across switches and
  avoids a second `SimulationState` hanging around unused.

### Files touched

- `run.sh` (new)
- `main.py` (defaults, remove `--interface`, expanded log)
- `core/api.py` (`home()`, `set_robot()`, private `_q_init`)
- `renderer/viewer.py` (new endpoints, extended meta, stores config/URDF path)
- `web/index.html` (6-card aside)
- `web/app.js` (new cards, keypad highlight, home / switch handlers)
- `web/styles.css` (card + keypad grid + joint bar styling)
- `README.md` (rewrite, below)
- `dev_log.md` (new, below)

### Files NOT touched

- `core/fk.py`, `core/ik.py`, `core/robot.py`, `core/state.py`
- `loader/urdf.py`
- `interface/keyboard.py`
- `configs/tr4.yaml`
- Any file under `third_party/` or `TR4_Pro/`
- Any test under `tests/` unless a new test is added; existing tests must still pass.

## C. Docs

### `README.md` (rewrite)

One page, four sections, no more:

1. **SimpleSim** — two to three sentences of philosophy: browser-based minimal
   runtime, URDF → render → keyboard IK → optional hardware bridge, no ROS, no
   physics, only control-logic verification.
2. **Quick start** — three commands, one port:
   ```
   git submodule update --init --recursive
   npm install
   ./run.sh
   ```
   Then `http://127.0.0.1:8765`.
3. **Controls** — small table: WASD translate XY, R/F translate Z, Q/E yaw,
   arrows pitch / roll.
4. **Layout** — six-line directory map (`core/ loader/ interface/ renderer/ web/ configs/`)
   with a one-liner per dir.

No API reference, no contribution guide, no license section.

### `dev_log.md` (new)

Top-of-file header explaining format: reverse-chronological, each entry has date,
short subject, bullet list of concrete changes, motivation line. First entry covers
this overhaul. Future entries are hand-written on each change — no git hooks, no
automation. Kept at project root so it's seen alongside README.

## Testing

Existing tests (`tests/test_loader.py`, `test_fk.py`, `test_ik.py`, `test_api.py`)
must continue to pass unchanged. `pytest tests/` is the gate before committing.

No new backend tests in this iteration — the new API endpoints are thin wrappers
over existing `move_joint` / URDF loading, both of which are already covered. If
`switch_arm` turns out to be load-bearing, it gets a test in a follow-up.

Manual verification checklist (run before committing):

1. `./run.sh` from a clean checkout on a fresh shell → server starts, browser
   loads, robot renders.
2. Hold W, A, R together → EE pose moves, keypad cells light up, joints table
   updates, IK dot stays green.
3. Click Home → joint table and EE pose return to initial values.
4. Click Arm toggle → right arm loads, robot re-renders around new base, keypad
   still drives the new active arm.
5. `pytest tests/` → green.

## Risks

- `switch_arm` re-loading the `RobotModel` replaces the URDF chain on the backend,
  but the frontend three.js scene already has the robot mesh attached. The
  frontend must re-run its `loadRobot()` path when meta changes. Plan must call
  this out as a concrete step and verify the scene replaces (not duplicates) the
  robot node.
- `run.sh` bash-isms on non-bash shells. Target is Linux / macOS with bash; fish
  and pwsh users fall back to running `python main.py` directly, which still
  works thanks to the default args. Not worth solving.
- Empty POST body on `/api/home`: `_json_body` already returns `{}` for zero
  content-length, so safe.

## Rollout

Single commit containing:

- `run.sh`, `main.py`, `core/api.py`, `renderer/viewer.py`, `web/*`, `README.md`,
  `dev_log.md`, and this spec.

Commit message: `ux: one-shot launcher, observable web panel, docs`.
