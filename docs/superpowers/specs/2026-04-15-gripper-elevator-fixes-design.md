# Gripper & Elevator Fixes — Design Spec

## Goal

Fix three runtime issues in SimpleSim's accessory controls for the TR4 Pro:
1. Elevator joint has wrong URDF limits and does not move.
2. Gripper linkage is visually "torn apart" due to incorrect coupling.
3. Gripper key semantics are backwards; two keys should become one toggle.

---

## Issue 1 — Elevator Not Working

### Root Cause

`URDFLoader` clamps every `setJointValue` call to `[limit.lower, limit.upper]`.
The `elevation` prismatic joint currently has incorrect limits (`lower=0 upper=0.35`
from a prior fix attempt). The physically correct range is `lower=-0.8 upper=0.3`,
giving 1.1 m of travel along the URDF Z-axis.

Additionally, Python's `SimpleHTTPServer` sets no `Cache-Control` headers, so the
browser may serve the stale URDF file even after the limits are corrected. We need
cache-busting on the URDF URL.

### Fix

**URDF** (`TR4_Pro/TR4_with_grippers_v2.urdf`):
```xml
<limit effort="0" lower="-0.8" upper="0.3" velocity="0"/>
```

**`configs/tr4.yaml`** — elevator section:
```json
"elevator": {
  "joint_name": "elevation",
  "lower": -0.8,
  "upper":  0.3,
  "init":   0.3,
  "step":   0.005
}
```
`init` is the starting joint value (robot begins at full height).

**`core/accessories.py`** — read `init` from config:
```python
self._elev_pos = float(elev.get("init", elev.get("upper", 0.0)))
```

**`renderer/viewer.py`** — append `?v=<timestamp>` to the URDF URL so the browser
never serves a cached file:
```python
import time
urdf_url_versioned = f"{self.urdf_url}?v={int(time.time())}"
# use urdf_url_versioned inside _meta_payload()
```

Key direction stays: `ArrowUp` → `step_elevator(+1)` (toward upper=0.3), `ArrowDown`
→ `step_elevator(-1)` (toward lower=-0.8).

---

## Issue 2 — Gripper Linkage Visual ("Torn Apart")

### Root Cause

Current approach: a single `coupling` vector multiplied by `aperture × max_angle`.
With all three joints sharing the same URDF axis `(0,0,-1)` and coupling `±1.0`,
every joint is driven to its individual limit simultaneously. The three-link finger
does not fold naturally — it visually explodes.

The correct mock model (Robotiq-style) specifies two explicit postures — **open** and
**closed** — and interpolates linearly. No physics required; each robot just supplies
its own pair of joint-value arrays in the config.

### Design

Replace `coupling`/`max_angle` with `open_positions` / `closed_positions` in YAML.

```
joint_value[i] = open[i] + aperture × (closed[i] - open[i])
```

`aperture` remains a scalar in `[0, 1]` (0 = open, 1 = closed).

**`configs/tr4.yaml`** — gripper section (example for right arm):
```json
"right": {
  "joint_names": [
    "right_gripper_l_joint1", "right_gripper_l_joint2", "right_gripper_l_joint3",
    "right_gripper_r_joint1", "right_gripper_r_joint2", "right_gripper_r_joint3"
  ],
  "open_positions":   [0.0,  0.0,  0.0,   0.0,  0.0,  0.0],
  "closed_positions": [-0.9, -0.7, -0.4,   0.9,  0.7,  0.4],
  "step": 1.0
}
```

Left arm: same pattern, limits sign matches URDF (`l_joint` upper=0, `r_joint` lower=0).

> **Tuning note:** `closed_positions` are initial guesses based on progressive
> finger curl. After testing, edit these six values in YAML to match the desired look.
> No code changes required.

`step` is now unused for continuous control (toggle jumps to 0 or 1 directly) but
kept for API compatibility.

**`core/accessories.py`** — `joint_values()` new formula:
```python
open_pos = g["open_positions"]
clos_pos = g["closed_positions"]
for i, jname in enumerate(g["joint_names"]):
    vals[jname] = open_pos[i] + aperture * (clos_pos[i] - open_pos[i])
```

---

## Issue 3 — G/B Key Semantics & Toggle

### Current behaviour (wrong)
- G → close (+1), B → open (-1), both held-down continuous keys.

### Desired behaviour
- Single **G** keypress toggles between fully open (`aperture=0`) and fully closed
  (`aperture=1`). Instant jump, no animation.
- B key removed entirely.

### Design

Add `toggle_gripper(arm_name)` to `AccessoryController`:
```python
def toggle_gripper(self, arm_name: str) -> None:
    with self._lock:
        g = self._grippers.get(arm_name)
        if g is None:
            return
        g["aperture"] = 0.0 if g["aperture"] > 0.5 else 1.0
```

In `renderer/viewer.py` keyboard handler, replace G/B continuous logic:
```python
if "KeyG" in keys:
    server_ref.accessory.toggle_gripper(server_ref.active_arm)
# remove KeyB handling
```

In `web/app.js`, move `KeyG` from `acceptedKeys` (continuous) to a one-shot handler
like `KeyR`/`KeyC` — fires once per keydown, does not accumulate in `activeKeys`.

In `web/index.html`, replace the two-button keypad group with a single wide key:
```html
<div class="key key-wide" data-code="KeyG">G &nbsp; toggle grip</div>
```
Remove the `KeyB` key element.

---

## Files Changed

| File | Change |
|------|--------|
| `TR4_Pro/TR4_with_grippers_v2.urdf` | Fix elevation limits |
| `configs/tr4.yaml` | elevator lower/upper/init; grippers open/closed positions |
| `core/accessories.py` | init from config; interpolation formula; toggle_gripper() |
| `renderer/viewer.py` | cache-bust URDF URL; toggle G, remove B |
| `web/app.js` | G as one-shot key; remove B from acceptedKeys |
| `web/index.html` | Single "toggle grip" key, remove B key |

---

## Out of Scope

- Smooth animation / tween for gripper (user confirmed instant jump is fine)
- Per-robot code paths (all config-driven)
- Physics simulation of contact compliance
