# SimpleSim

A minimal browser-based robot runtime. Load a URDF, render it in 3D, and drive the end-effector with keyboard-controlled IK — no ROS, no physics engine, no vendor SDK requirement.

<p align="center">
  <img src="media/demo.gif" alt="SimpleSim demo" width="820">
</p>

## Quick Start

```bash
git submodule update --init --recursive
npm install
./run.sh
```

Open <http://127.0.0.1:8765> in your browser.

## Supported Robot Configs

| Robot | URDF | Config |
|------|------|--------|
| TR4 Pro dual-arm | `third_party/TR4_Pro/TR4_with_grippers_v2.urdf` | `configs/tr4.yaml` |
| Franka FR3 + hand | `third_party/franka_description/simplesim/fr3_with_franka_hand.urdf` | `configs/fr3.yaml` |

Launch examples:

```bash
python main.py --urdf third_party/TR4_Pro/TR4_with_grippers_v2.urdf --config configs/tr4.yaml
python main.py --urdf third_party/franka_description/simplesim/fr3_with_franka_hand.urdf --config configs/fr3.yaml
```

## Controls

| Action | Key(s) |
|--------|--------|
| Translate forward / back | `W` / `S` |
| Translate left / right | `A` / `D` |
| Translate up / down | `Q` / `E` |
| Roll | `J` / `U` |
| Pitch | `K` / `I` |
| Yaw | `L` / `O` |
| Linear axis up / down | `↑` / `↓` |
| Toggle gripper open/close | `G` |
| Reset to home pose | `R` |
| Switch active arm | `C` |

## Tests

```bash
python -m pytest tests/
node --test web/panel-model.test.mjs
```
