# SimpleSim

A minimal browser-based robot runtime. Load a URDF, render it in 3D, drive the end-effector with keyboard-controlled IK, and optionally bridge to real hardware — no ROS, no physics engine, no overhead.

## Quick Start

```bash
git submodule update --init --recursive
npm install
./run.sh
```

Open <http://127.0.0.1:8765> in your browser.

`run.sh` checks for required submodules, `node_modules/three`, and Python `numpy`. It reports what is missing without installing anything automatically.

## Controls

| Action | Key(s) |
|--------|--------|
| Translate forward / back | `W` / `S` |
| Translate left / right | `A` / `D` |
| Translate up / down | `Q` / `E` |
| Roll | `J` / `U` |
| Pitch | `K` / `I` |
| Yaw | `L` / `O` |
| Elevator up / down | `↑` / `↓` |
| Toggle gripper open/close | `G` |
| Reset to home pose | `R` |
| Switch active arm | `C` |

All translation and rotation deltas are expressed in the robot frame and converted to URDF world frame before the IK step, so controls behave consistently regardless of which arm is active.

## Side Panel

| Card | Contents |
|------|----------|
| **Robot** | Robot name, active kinematic chain (base → ee link), active arm badge |
| **Keypad** | Visual key highlights — keys light up while held |
| **EE Pose** | End-effector position (x, y, z) and orientation (roll, pitch, yaw) |
| **Joints** | Joint angles in radians and degrees, with range bar |
| **IK** | Last solver result: iteration count, position error, orientation error |
| **Accessories** | Elevator height (m) and per-arm gripper state (open / closed) |

Each arm maintains its own joint state independently; switching arms preserves the previous arm's pose.

## Configuration

All runtime parameters live in `configs/tr4.yaml` (JSON format).

| Section | Description |
|---------|-------------|
| `arms` | Per-arm base link, ee link, joint names, and initial joint angles |
| `ik` | DLS damping, max iterations, position/orientation tolerances, step limits |
| `elevator` | Joint name, travel range (`lower`, `upper`), startup position (`init`), step size |
| `grippers` | Per-arm joint names, `open_positions`, and `closed_positions` (rad) |
| `keyboard` | Translation step (m) and rotation step (rad) per tick |
| `server` | Host, port, and state poll interval |
| `rm_api` | Real-hardware bridge parameters (disabled by default) |

Gripper postures are defined as explicit joint-angle vectors. The runtime linearly interpolates between `open_positions` and `closed_positions` — tuning the gripper appearance only requires editing these values in the config file, no code changes needed.

## Architecture

```
core/        # RobotModel, FK (world frame), DLS IK, SimulationState, RobotAPI, AccessoryController
loader/      # URDF parser → RobotModel (resolves base-link world transform)
interface/   # Keyboard bindings → EE delta (robot frame → URDF world frame)
renderer/    # HTTP server, /api endpoints, static file serving
web/         # Three.js + URDFLoader frontend
configs/     # Runtime configuration files
tests/       # pytest unit tests
```

`interface` and `renderer` never mutate simulation state directly — all state changes go through `core/api.py`.

## Tests

```bash
python -m pytest tests/
```
