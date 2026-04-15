# Accessories Status Card Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the "Actions" card (Home/Switch Arm buttons) with an "Accessories" card showing live elevator height and per-arm gripper open/closed state.

**Architecture:** Server attaches `accessory` snapshot to every state response. Frontend reads it in `applyState` and renders a new `card-accessories` section. Button DOM refs and listeners are removed.

**Tech Stack:** Python http.server, vanilla JS ES modules, HTML.

**Spec:** `docs/superpowers/specs/2026-04-15-accessories-card-design.md`

---

### Task 1: Attach accessory snapshot to every state response

**Files:**
- Modify: `renderer/viewer.py` — `_with_all_joints` method (~line 91)

- [ ] **Step 1: Add `accessory` field in `_with_all_joints`**

Open `renderer/viewer.py`. Find `_with_all_joints`:

```python
def _with_all_joints(self, state: Dict[str, Any]) -> Dict[str, Any]:
    state["all_joints"] = self._all_joints()
    return state
```

Replace with:

```python
def _with_all_joints(self, state: Dict[str, Any]) -> Dict[str, Any]:
    state["all_joints"] = self._all_joints()
    state["accessory"] = self.accessory.snapshot()
    return state
```

`accessory.snapshot()` already exists in `core/accessories.py` and returns:
```python
{"elevator": float, "grippers": {"left": float, "right": float}}
```
where gripper values are aperture in [0.0 = open, 1.0 = closed].

- [ ] **Step 2: Verify by curl**

Start the server (`python main.py`), then:

```bash
curl -s http://127.0.0.1:8765/api/state | python3 -m json.tool | grep -A 5 accessory
```

Expected output includes:
```json
"accessory": {
    "elevator": -0.3,
    "grippers": {"left": 0.0, "right": 0.0}
}
```

- [ ] **Step 3: Commit**

```bash
git add renderer/viewer.py
git commit -m "feat: attach accessory snapshot to every state response"
```

---

### Task 2: Replace Actions card with Accessories card in HTML

**Files:**
- Modify: `web/index.html` — `card-actions` section (~lines 124–130)

- [ ] **Step 1: Replace the card-actions section**

Find and remove the entire `card-actions` section:

```html
<section class="card" id="card-actions">
  <h2>Actions</h2>
  <div class="actions">
    <button type="button" id="btn-home">Home</button>
    <button type="button" id="btn-arm">Switch Arm</button>
  </div>
</section>
```

Replace with:

```html
<section class="card" id="card-accessories">
  <h2>Accessories</h2>
  <dl class="pose">
    <dt>Elevation</dt><dd id="acc-elevation">-</dd>
    <dt>Grip L</dt>   <dd id="acc-grip-l">-</dd>
    <dt>Grip R</dt>   <dd id="acc-grip-r">-</dd>
  </dl>
</section>
```

This reuses the existing `.pose` dl style (same as the EE Pose card) so no CSS changes are needed.

- [ ] **Step 2: Commit**

```bash
git add web/index.html
git commit -m "feat: replace Actions card with Accessories status card"
```

---

### Task 3: Wire up accessory display in app.js, remove button code

**Files:**
- Modify: `web/app.js`

- [ ] **Step 1: Replace button DOM refs with accessory refs**

Find the existing button refs near the top of `app.js`:

```javascript
const homeBtn = document.getElementById('btn-home');
const armBtn = document.getElementById('btn-arm');
```

Replace with:

```javascript
const accElevationEl = document.getElementById('acc-elevation');
const accGripLEl     = document.getElementById('acc-grip-l');
const accGripREl     = document.getElementById('acc-grip-r');
```

- [ ] **Step 2: Add `applyAccessory` function**

After the existing `applyIk` function, add:

```javascript
function applyAccessory(accessory) {
  if (!accessory) return;
  accElevationEl.textContent = fmt(accessory.elevator, 3) + ' m';
  accGripLEl.textContent  = (accessory.grippers?.left  ?? 0) >= 0.5 ? 'closed' : 'open';
  accGripREl.textContent  = (accessory.grippers?.right ?? 0) >= 0.5 ? 'closed' : 'open';
}
```

- [ ] **Step 3: Call `applyAccessory` from `applyState`**

In `applyState`, after the existing `if (state.ik ...)` line, add:

```javascript
if (state.accessory) applyAccessory(state.accessory);
```

- [ ] **Step 4: Remove button event listeners**

Find and delete these two lines in `setupKeyboard`:

```javascript
homeBtn.addEventListener('click', doHome);
armBtn.addEventListener('click', doSwitchArm);
```

- [ ] **Step 5: Manual smoke-test**

Restart the server, hard-refresh the browser (`Ctrl+Shift+R`).
- The side panel should show "Accessories" card with Elevation and Grip L/R rows.
- The "Actions" card with Home/Switch Arm buttons should be gone.
- Press `ArrowUp` — Elevation value should change.
- Press `G` — Grip R (or L, whichever is active arm) should toggle between "open" and "closed".
- Press `C` to switch arms — no camera jump, Grip label should reflect active arm.

- [ ] **Step 6: Commit**

```bash
git add web/app.js
git commit -m "feat: wire accessory card display, remove Action button listeners"
```
