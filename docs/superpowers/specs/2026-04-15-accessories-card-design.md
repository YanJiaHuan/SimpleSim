# Accessories Status Card — Design Spec

## Goal

Replace the "Actions" card in the side panel with an "Accessories" status card that
shows real-time elevator height and per-arm gripper open/closed state.

---

## What Changes

### Remove

- `card-actions` HTML section (Home / Switch Arm buttons)
- `homeBtn` / `armBtn` DOM refs and their `addEventListener` calls in `app.js`
- `#btn-home` / `#btn-arm` element queries in `app.js`

Home and Switch Arm remain accessible via keyboard keys R and C.

### Add

**Server — `renderer/viewer.py`**

`_with_all_joints()` also attaches the accessory snapshot:

```python
def _with_all_joints(self, state):
    state["all_joints"] = self._all_joints()
    state["accessory"] = self.accessory.snapshot()
    return state
```

`AccessoryController.snapshot()` already returns:
```python
{"elevator": float, "grippers": {"left": float, "right": float}}
```
where gripper values are `aperture` in [0, 1].

**Frontend — `web/index.html`**

New card replacing `card-actions`:
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

**Frontend — `web/app.js`**

New DOM refs:
```javascript
const accElevationEl = document.getElementById('acc-elevation');
const accGripLEl     = document.getElementById('acc-grip-l');
const accGripREl     = document.getElementById('acc-grip-r');
```

New `applyAccessory(accessory)` function called from `applyState`:
```javascript
function applyAccessory(accessory) {
  if (!accessory) return;
  accElevationEl.textContent = fmt(accessory.elevator, 3) + ' m';
  accGripLEl.textContent = accessory.grippers?.left  >= 0.5 ? 'closed' : 'open';
  accGripREl.textContent = accessory.grippers?.right >= 0.5 ? 'closed' : 'open';
}
```

In `applyState`:
```javascript
if (state.accessory) applyAccessory(state.accessory);
```

---

## Files Changed

| File | Change |
|------|--------|
| `renderer/viewer.py` | `_with_all_joints` attaches `accessory` snapshot |
| `web/index.html` | Replace `card-actions` with `card-accessories` |
| `web/app.js` | Add accessory DOM refs + `applyAccessory`; remove button refs/listeners |

---

## Out of Scope

- Partial aperture percentage display (open/closed binary is sufficient)
- Styling changes beyond reusing existing `.pose` dl pattern
