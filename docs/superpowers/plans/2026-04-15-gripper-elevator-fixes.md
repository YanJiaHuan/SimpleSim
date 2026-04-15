# Gripper & Elevator Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix elevator not moving, gripper linkage "torn apart" visual, and G/B key semantics.

**Architecture:** Three independent fixes touching URDF, config, one Python module, one Python server file, and two frontend files. No new files needed.

**Tech Stack:** Python (http.server), JavaScript (ES modules, Three.js URDFLoader), JSON config.

**Spec:** `docs/superpowers/specs/2026-04-15-gripper-elevator-fixes-design.md`

---

### Task 1: Fix elevator URDF limits + cache-busting

**Files:**
- Modify: `TR4_Pro/TR4_with_grippers_v2.urdf` line 53
- Modify: `configs/tr4.yaml` (elevator section)
- Modify: `core/accessories.py` (init value)
- Modify: `renderer/viewer.py` (_meta_payload)

- [ ] **Step 1: Fix URDF elevation limit**

Open `TR4_Pro/TR4_with_grippers_v2.urdf`, find line 53 (inside `<joint name="elevation">`):

```xml
<!-- Change this: -->
<limit effort="0" lower="0" upper="0.35" velocity="0"/>
<!-- To this: -->
<limit effort="0" lower="0" upper="0.8" velocity="0"/>
```

- [ ] **Step 2: Fix config elevator section**

In `configs/tr4.yaml`, replace the `"elevator"` block:

```json
"elevator": {
  "joint_name": "elevation",
  "lower": 0.0,
  "upper": 0.8,
  "init":  0.3,
  "step":  0.005
},
```

- [ ] **Step 3: Read `init` from config in AccessoryController**

In `core/accessories.py` `__init__`, replace:
```python
self._elev_pos: float = self._elev_lower
```
With:
```python
self._elev_pos: float = float(elev.get("init", self._elev_lower))
```
(Keep after `_elev_lower` and `_elev_upper` are assigned so clamping is available.)

Also clamp init to valid range right after:
```python
self._elev_pos = max(self._elev_lower, min(self._elev_upper, self._elev_pos))
```

- [ ] **Step 4: Add cache-busting to URDF URL**

In `renderer/viewer.py`, add `import time` at the top (with other stdlib imports).

In `_meta_payload`, change:
```python
"urdf_url": self.urdf_url,
```
To:
```python
"urdf_url": f"{self.urdf_url}?v={int(time.time())}",
```

- [ ] **Step 5: Manual smoke-test**

Start the server (`python main.py`), open the browser, hard-refresh (`Ctrl+Shift+R`).
Press `ArrowUp` several times — torso should visibly rise. Press `ArrowDown` — torso
should lower. Confirm it stops at the physical limits (not shooting through the floor).

- [ ] **Step 6: Commit**

```bash
git add TR4_Pro/TR4_with_grippers_v2.urdf configs/tr4.yaml core/accessories.py renderer/viewer.py
git commit -m "fix: elevator limits (0-0.8), init=0.3, cache-bust URDF URL"
```

---

### Task 2: Fix gripper linkage — open/closed position interpolation

**Files:**
- Modify: `configs/tr4.yaml` (grippers section)
- Modify: `core/accessories.py` (joint_values, __init__)

- [ ] **Step 1: Update config grippers section**

Replace the entire `"grippers"` block in `configs/tr4.yaml` with:

```json
"grippers": {
  "left": {
    "joint_names": [
      "left_gripper_l_joint1", "left_gripper_l_joint2", "left_gripper_l_joint3",
      "left_gripper_r_joint1", "left_gripper_r_joint2", "left_gripper_r_joint3"
    ],
    "open_positions":   [0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
    "closed_positions": [-0.9, -0.7, -0.4,  0.9,  0.7,  0.4]
  },
  "right": {
    "joint_names": [
      "right_gripper_l_joint1", "right_gripper_l_joint2", "right_gripper_l_joint3",
      "right_gripper_r_joint1", "right_gripper_r_joint2", "right_gripper_r_joint3"
    ],
    "open_positions":   [0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
    "closed_positions": [-0.9, -0.7, -0.4,  0.9,  0.7,  0.4]
  }
},
```

(Remove `"coupling"`, `"max_angle"`, and `"step"` fields.)

- [ ] **Step 2: Update AccessoryController.__init__ to load open/closed positions**

In `core/accessories.py`, inside the `for arm_name, gcfg in gripper_cfg.items():` loop,
replace:
```python
self._grippers[arm_name] = {
    "joint_names": [str(n) for n in gcfg.get("joint_names", [])],
    "coupling": [float(c) for c in gcfg.get("coupling", [])],
    "max_angle": float(gcfg.get("max_angle", 1.0472)),
    "step": float(gcfg.get("step", 0.05)),
    "aperture": 0.0,
}
```
With:
```python
joint_names = [str(n) for n in gcfg.get("joint_names", [])]
n = len(joint_names)
self._grippers[arm_name] = {
    "joint_names": joint_names,
    "open_positions":   [float(v) for v in gcfg.get("open_positions",   [0.0] * n)],
    "closed_positions": [float(v) for v in gcfg.get("closed_positions", [0.0] * n)],
    "aperture": 0.0,
}
```

- [ ] **Step 3: Update joint_values() to use interpolation**

In `core/accessories.py`, replace the gripper loop inside `joint_values()`:
```python
# old:
for i, jname in enumerate(g["joint_names"]):
    coupling = g["coupling"][i] if i < len(g["coupling"]) else 1.0
    vals[jname] = coupling * aperture * max_angle
```
With:
```python
# new:
open_pos = g["open_positions"]
clos_pos = g["closed_positions"]
for i, jname in enumerate(g["joint_names"]):
    vals[jname] = open_pos[i] + aperture * (clos_pos[i] - open_pos[i])
```

Also remove `max_angle = g["max_angle"]` from the same function (no longer needed).

- [ ] **Step 4: Manual smoke-test**

Restart the server, open browser.
Press `G` — gripper should close into a curled-finger shape (not explode).
Press `G` again — gripper should open back to flat.
Visually confirm both arms' grippers move symmetrically.

If the closed shape looks wrong (e.g. fingers still cross or gap too wide), edit
`closed_positions` in `configs/tr4.yaml` and restart — no code changes needed.

- [ ] **Step 5: Commit**

```bash
git add configs/tr4.yaml core/accessories.py
git commit -m "fix: gripper linkage — replace coupling with open/closed position interpolation"
```

---

### Task 3: G key toggle + remove B key

**Files:**
- Modify: `core/accessories.py` (add toggle_gripper)
- Modify: `renderer/viewer.py` (keyboard handler)
- Modify: `web/app.js` (G as one-shot, remove B)
- Modify: `web/index.html` (UI update)

- [ ] **Step 1: Add toggle_gripper to AccessoryController**

In `core/accessories.py`, after `step_gripper`, add:

```python
def toggle_gripper(self, arm_name: str) -> None:
    """Toggle gripper between fully open (aperture=0) and fully closed (aperture=1)."""
    with self._lock:
        g = self._grippers.get(arm_name)
        if g is None:
            return
        g["aperture"] = 0.0 if g["aperture"] > 0.5 else 1.0
```

- [ ] **Step 2: Update keyboard handler in viewer.py**

In `renderer/viewer.py`, inside `do_POST` for `path == "/api/keyboard"`, replace:
```python
if "KeyG" in keys:
    server_ref.accessory.step_gripper(server_ref.active_arm, +1)
if "KeyB" in keys:
    server_ref.accessory.step_gripper(server_ref.active_arm, -1)
```
With:
```python
if "KeyG" in keys:
    server_ref.accessory.toggle_gripper(server_ref.active_arm)
```

- [ ] **Step 3: Update app.js — G as one-shot, remove B**

In `web/app.js`:

1. Remove `'KeyB'` from `acceptedKeys`:
```javascript
const acceptedKeys = new Set([
  'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE',
  'KeyJ', 'KeyU', 'KeyK', 'KeyI', 'KeyL', 'KeyO',
  'ArrowUp', 'ArrowDown',
]);
```

2. Add `'KeyG'` to the one-shot constant alongside `switchArmKey` and `resetKey`.
Add near the top where those are declared:
```javascript
const gripperKey = 'KeyG';
```

3. In the `keydown` handler, add a new one-shot block before the `acceptedKeys` check:
```javascript
if (event.code === gripperKey) {
  event.preventDefault();
  const el = keyEls.get(gripperKey);
  el?.classList.add('active');
  fetch('/api/keyboard', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ keys: ['KeyG'] }),
  })
    .then(r => r.json())
    .then(state => { applyState(state); })
    .finally(() => el?.classList.remove('active'));
  return;
}
```

- [ ] **Step 4: Update index.html — single toggle key, remove B**

In `web/index.html`, find the Gripper keypad-group and replace the two keys:
```html
<div class="keypad-group">
  <p class="keypad-label">Gripper</p>
  <div class="keypad">
    <div class="keypad-row">
      <div class="key key-wide" data-code="KeyG">G &nbsp; toggle grip</div>
    </div>
  </div>
</div>
```

- [ ] **Step 5: Manual smoke-test**

Restart server and refresh browser.
- Press `G` once — gripper closes instantly.
- Press `G` again — gripper opens instantly.
- `B` key should have no effect.
- The keypad panel shows only the single "G toggle grip" button.

- [ ] **Step 6: Commit**

```bash
git add core/accessories.py renderer/viewer.py web/app.js web/index.html
git commit -m "fix: G key toggles gripper open/close, remove B key"
```
