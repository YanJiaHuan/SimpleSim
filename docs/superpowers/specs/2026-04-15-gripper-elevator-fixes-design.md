# Gripper & Elevator Fixes — Design Spec

## Goal

Fix accessory control issues in SimpleSim for the TR4 Pro:
1. Elevator joint has wrong URDF limits and does not move.
2. Gripper linkage is visually "torn apart" — joint3 should counter-rotate.
3. Gripper key semantics: two keys → single G toggle.
4. Arm switch camera jump (regression from cache-busting timestamp fix).

---

## Issue 1 — Elevator Wrong Limits

### Root Cause

`URDFLoader` clamps every `setJointValue` call to `[limit.lower, limit.upper]`.
The `elevation` prismatic joint must have `lower=-0.8 upper=0`, with axis `(0,0,1)`:
- joint=0: torso at highest position (retracted)
- joint=-0.8: torso at lowest position (extended down)

init=-0.3 starts the robot 0.3 m below the top.

Additionally, Python's `SimpleHTTPServer` sets no `Cache-Control` headers, so the
browser may serve a stale URDF. We need cache-busting on the URDF URL — computed
**once at server startup** (see Issue 4).

### Fix

**URDF** (`TR4_Pro/TR4_with_grippers_v2.urdf`):
```xml
<limit effort="0" lower="-0.8" upper="0" velocity="0"/>
```

**`configs/tr4.yaml`** — elevator section:
```json
"elevator": {
  "joint_name": "elevation",
  "lower": -0.8,
  "upper":  0.0,
  "init":  -0.3,
  "step":   0.005
}
```

Key direction: `ArrowUp` → `step_elevator(+1)` toward upper=0 (physically rising),
`ArrowDown` → `step_elevator(-1)` toward lower=-0.8 (physically lowering).

---

## Issue 2 — Gripper Linkage ("Torn Apart")

### Root Cause

Previous fix drove all three joints proportionally in the same direction. The physical
design (Robotiq-style underactuated finger) requires joint3 (the back linkage) to
**counter-rotate** relative to joint1/joint2:
- joint1 (palm knuckle): main driver, closes inward
- joint2 (finger): follows joint1 (same direction)
- joint3 (linkage/tendon): moves in the **opposite** direction — returns toward 0 from
  a pre-loaded negative offset as the gripper closes

This matches the URDF joint limits: all l_joints are `[-1.0472, 0]` so joint3 can
travel from -0.5 (open, pre-loaded) back to 0 (closed), which is valid and opposite
to joint1's motion.

### Fix

**`configs/tr4.yaml`** — grippers section (same for both arms, same URDF):
```json
"grippers": {
  "left": {
    "joint_names": ["left_gripper_l_joint1", "left_gripper_l_joint2", "left_gripper_l_joint3",
                    "left_gripper_r_joint1", "left_gripper_r_joint2", "left_gripper_r_joint3"],
    "open_positions":   [0.0,  0.0, -0.5,  0.0,  0.0,  0.5],
    "closed_positions": [-0.8, -0.6,  0.0,  0.8,  0.6,  0.0]
  },
  "right": { /* identical */ }
}
```

Linear interpolation in `accessories.py`:
```
joint_value[i] = open[i] + aperture × (closed[i] - open[i])
```

joint3 (index 2): open=-0.5 → closed=0 (positive travel = counter-rotation).
joint3 (index 5, r_side): open=0.5 → closed=0 (negative travel = counter-rotation).

> **Tuning:** if closed shape looks wrong, edit `closed_positions` in YAML and restart.

---

## Issue 3 — G/B Key Semantics & Toggle (already implemented)

- Single G keypress toggles aperture: 0→1 (open→closed) or 1→0 (closed→open).
- B key removed.
- Implemented in prior commit; no changes needed.

---

## Issue 4 — Arm Switch Camera Jump (regression)

### Root Cause

`_meta_payload()` uses `f"{self.urdf_url}?v={int(time.time())}"`, generating a **new
timestamp on every call**. `doSwitchArm` in `app.js` compares `prevUrdfUrl` to
`meta.urdf_url` and calls `reloadRobot()` when they differ — which is always true now
because the timestamp changed between the two `/api/meta` or `/api/switch_arm` calls.

### Fix

Compute the version stamp **once** in `RuntimeServer.__init__`:
```python
import time
self._urdf_version = int(time.time())
```

In `_meta_payload`:
```python
"urdf_url": f"{self.urdf_url}?v={self._urdf_version}",
```

This keeps the URL stable across arm switches while still cache-busting on server restart.

---

## Files Changed

| File | Change |
|------|--------|
| `TR4_Pro/TR4_with_grippers_v2.urdf` | elevation limits: lower=-0.8 upper=0 |
| `configs/tr4.yaml` | elevator lower/upper/init; grippers open/closed with joint3 counter-rotate |
| `renderer/viewer.py` | timestamp computed once in `__init__`, not per call |

`core/accessories.py` and frontend files require no changes (already correct).

---

## Out of Scope

- Smooth animation/tween for gripper
- Per-robot code paths (all config-driven)
- Contact compliance simulation
