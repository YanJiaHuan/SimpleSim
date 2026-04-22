# FR3 Capability Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add offline `FR3 + franka_hand` support, remove RM real-hardware bridge code, and make SimpleSim's runtime/UI capability-driven so TR4 Pro and FR3 both work from config without TR4-specific assumptions.

**Architecture:** Keep the existing in-repo URDF loader, FK, and DLS IK as the only runtime backend. Generalize robot-specific behavior through explicit arm/accessory config plus `/api/meta` capability data, then render the web panel from that contract instead of hard-coded TR4 controls.

**Tech Stack:** Python (`http.server`, pytest), JavaScript (ES modules, `node --test`, Three.js + URDFLoader), JSON-formatted YAML configs, git submodules.

**Spec:** `docs/superpowers/specs/2026-04-17-fr3-capability-runtime-design.md`

---

### Task 1: Normalize third-party robot assets and add FR3 entrypoint

**Files:**
- Create: `tests/test_robot_assets.py`
- Modify: `.gitmodules`
- Modify: `run.sh`
- Move: `TR4_Pro/` -> `third_party/TR4_Pro/`
- Create: `third_party/franka_description/simplesim/fr3_with_franka_hand.urdf`
- Remove: `third_party/RM_API2/`

- [ ] **Step 1: Write the failing asset-layout regression test**

Create `tests/test_robot_assets.py`:

```python
from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_expected_robot_asset_entrypoints_exist() -> None:
    assert (ROOT / "third_party" / "TR4_Pro" / "TR4_with_grippers_v2.urdf").is_file()
    assert (
        ROOT
        / "third_party"
        / "franka_description"
        / "simplesim"
        / "fr3_with_franka_hand.urdf"
    ).is_file()


def test_gitmodules_contains_franka_and_not_rm_api2() -> None:
    gitmodules = (ROOT / ".gitmodules").read_text(encoding="utf-8")
    assert '[submodule "third_party/franka_description"]' in gitmodules
    assert '[submodule "third_party/RM_API2"]' not in gitmodules
```

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```bash
python -m pytest tests/test_robot_assets.py -v
```

Expected: FAIL because `third_party/TR4_Pro/...` and `third_party/franka_description/simplesim/...` do not exist yet, and `.gitmodules` still contains `third_party/RM_API2`.

- [ ] **Step 3: Update submodule definitions**

Edit `.gitmodules` to remove the RM API2 stanza and add Franka:

```ini
[submodule "third_party/urdf-loaders"]
	path = third_party/urdf-loaders
	url = git@github.com:YanJiaHuan/urdf-loaders.git
[submodule "third_party/franka_description"]
	path = third_party/franka_description
	url = https://github.com/frankarobotics/franka_description.git
```

Also convert the current top-level `TR4_Pro/` asset tree into a dedicated repository and re-add it at `third_party/TR4_Pro/` as a submodule before continuing. The parent repo should no longer contain a top-level `TR4_Pro/` directory when this task is done.

- [ ] **Step 4: Materialize the local FR3 URDF entrypoint**

Create `third_party/franka_description/simplesim/fr3_with_franka_hand.urdf` from the official `fr3` + `franka_hand` description, with mesh paths kept relative to `third_party/franka_description`.

The generated URDF must expose the standard prefixed FR3 chain:

```xml
<robot name="fr3">
  <link name="fr3_link0"/>
  <joint name="fr3_joint1" type="revolute">...</joint>
  <joint name="fr3_joint2" type="revolute">...</joint>
  <joint name="fr3_joint3" type="revolute">...</joint>
  <joint name="fr3_joint4" type="revolute">...</joint>
  <joint name="fr3_joint5" type="revolute">...</joint>
  <joint name="fr3_joint6" type="revolute">...</joint>
  <joint name="fr3_joint7" type="revolute">...</joint>
  <link name="fr3_hand"/>
  <link name="fr3_hand_tcp"/>
</robot>
```

Do not make SimpleSim depend on ROS or xacro at runtime. Commit the plain URDF artifact.

- [ ] **Step 5: Update `run.sh` to validate the new required asset entrypoints**

Replace the current submodule guard in `run.sh` with:

```bash
[[ -f third_party/urdf-loaders/javascript/src/URDFLoader.js ]] \
  || die "missing submodule — run: git submodule update --init --recursive"

[[ -f third_party/TR4_Pro/TR4_with_grippers_v2.urdf ]] \
  || die "missing TR4 asset submodule — run: git submodule update --init --recursive"

[[ -f third_party/franka_description/simplesim/fr3_with_franka_hand.urdf ]] \
  || die "missing Franka description assets — run: git submodule update --init --recursive"
```

- [ ] **Step 6: Re-run the asset test**

Run:

```bash
python -m pytest tests/test_robot_assets.py -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add .gitmodules run.sh tests/test_robot_assets.py third_party/franka_description third_party/TR4_Pro
git commit -m "chore: normalize robot assets under third_party"
```

---

### Task 2: Remove RM bridge code from the runtime path

**Files:**
- Modify: `core/api.py`
- Modify: `main.py`
- Modify: `tests/test_api.py`
- Modify: `README.md`

- [ ] **Step 1: Write the failing API regression test for bridge-free responses**

Append to `tests/test_api.py`:

```python
def test_move_joint_response_has_no_rm_bridge_fields() -> None:
    robot = make_planar_robot()
    api = RobotAPI(robot=robot, state=make_initial_state(robot))

    out = api.move_joint([0.2, -0.2])

    assert out["success"] is True
    assert "rm_ret" not in out
```

- [ ] **Step 2: Run the focused API test and verify it fails**

Run:

```bash
python -m pytest tests/test_api.py::test_move_joint_response_has_no_rm_bridge_fields -v
```

Expected: FAIL because `RobotAPI.move_joint()` currently includes `"rm_ret"` in the response.

- [ ] **Step 3: Remove RM bridge support from `core/api.py`**

Delete `RMArmBridge` entirely and simplify `RobotAPI` so it no longer accepts or stores `rm_bridge`.

The constructor signature should become:

```python
class RobotAPI:
    def __init__(
        self,
        robot: RobotModel,
        state: SimulationState,
        ik_config: Dict[str, Any] | None = None,
        q_init: List[float] | None = None,
    ) -> None:
        self.robot = robot
        self.state = state
        self.ik_config = ik_config or {}
        self._lock = threading.RLock()
        self._q_init = list(q_init) if q_init is not None else list(state.q)
```

In both `move_joint()` and `move_ee()`, remove the `rm_ret` branch and return only the simulation snapshot plus success/IK fields.

- [ ] **Step 4: Remove RM setup from `main.py` and update defaults**

In `main.py`:

1. Remove the `RMArmBridge` import.
2. Delete `rm_bridge = RMArmBridge(...)`.
3. Construct `RobotAPI` without `rm_bridge`.
4. Change the default URDF path to the new third-party location:

```python
parser.add_argument(
    "--urdf",
    default="third_party/TR4_Pro/TR4_with_grippers_v2.urdf",
    help="Path to URDF file (default: third_party/TR4_Pro/TR4_with_grippers_v2.urdf)",
)
```

5. Remove the RM status print block entirely.

- [ ] **Step 5: Update README wording to remove real-hardware claims**

In `README.md`, replace the opening paragraph:

```md
A minimal browser-based robot runtime. Load a URDF, render it in 3D, drive the end-effector with keyboard-controlled IK, and optionally bridge to real hardware — no ROS, no physics engine, no overhead.
```

With:

```md
A minimal browser-based robot runtime. Load a URDF, render it in 3D, and drive the end-effector with keyboard-controlled IK — no ROS, no physics engine, no vendor SDK requirement.
```

Also remove the `rm_api` row from the configuration table.

- [ ] **Step 6: Re-run API tests**

Run:

```bash
python -m pytest tests/test_api.py -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add core/api.py main.py tests/test_api.py README.md
git commit -m "refactor: remove RM bridge from runtime"
```

---

### Task 3: Replace TR4-specific accessories with explicit semantic accessory config

**Files:**
- Create: `tests/test_accessories.py`
- Modify: `core/accessories.py`
- Modify: `configs/tr4.yaml`
- Modify: `renderer/viewer.py`

- [ ] **Step 1: Write the failing accessory-controller test**

Create `tests/test_accessories.py`:

```python
from __future__ import annotations

from core.accessories import AccessoryController


def test_accessory_controller_reports_semantic_snapshot_and_joint_values() -> None:
    controller = AccessoryController(
        {
            "accessories": [
                {
                    "id": "column",
                    "kind": "linear_axis",
                    "label": "Elevation",
                    "joint_name": "elevation",
                    "lower": -0.8,
                    "upper": 0.0,
                    "init": -0.3,
                    "step": 0.005,
                },
                {
                    "id": "right_gripper",
                    "kind": "gripper",
                    "label": "Right Grip",
                    "arm": "right",
                    "joint_names": ["finger_a", "finger_b"],
                    "open_positions": [0.04, 0.04],
                    "closed_positions": [0.0, 0.0],
                },
            ]
        }
    )

    controller.apply_keys(["ArrowUp"], active_arm="right")
    controller.apply_keys(["KeyG"], active_arm="right")

    snapshot = controller.snapshot()
    joint_values = controller.joint_values()

    assert snapshot["column"]["kind"] == "linear_axis"
    assert snapshot["column"]["value"] == -0.295
    assert snapshot["right_gripper"]["kind"] == "gripper"
    assert snapshot["right_gripper"]["is_closed"] is True
    assert joint_values["elevation"] == -0.295
    assert joint_values["finger_a"] == 0.0
    assert joint_values["finger_b"] == 0.0
```

- [ ] **Step 2: Write the failing runtime-meta test**

At the top of `tests/test_api.py`, add:

```python
from pathlib import Path
```

Then append to `tests/test_api.py`:

```python
from core.accessories import AccessoryController
from interface.keyboard import KeyboardInterface
from renderer.viewer import RuntimeServer


def test_runtime_meta_reports_capabilities_from_config() -> None:
    robot = make_planar_robot()
    server = RuntimeServer(
        api=RobotAPI(robot=robot, state=make_initial_state(robot)),
        keyboard=KeyboardInterface(),
        accessory=AccessoryController(
            {
                "accessories": [
                    {
                        "id": "hand",
                        "kind": "gripper",
                        "label": "Hand",
                        "arm": "main",
                        "joint_names": ["finger_joint1", "finger_joint2"],
                        "open_positions": [0.04, 0.04],
                        "closed_positions": [0.0, 0.0],
                    }
                ]
            }
        ),
        static_root=Path("."),
        host="127.0.0.1",
        port=8765,
        urdf_url="/robot.urdf",
        arms_data={"main": (robot, make_initial_state(robot), [0.0, 0.0])},
        config={
            "robot_label": "Planar",
            "arms": {
                "main": {
                    "label": "Main Arm",
                    "base_link": "base",
                    "ee_link": "ee",
                    "joint_names": ["joint1", "joint2"],
                    "q_init": [0.0, 0.0],
                }
            },
        },
        active_arm="main",
    )

    meta = server._meta_payload()

    assert meta["controls"]["can_switch_arm"] is False
    assert meta["controls"]["show_linear_axis_controls"] is False
    assert meta["panels"]["show_arm_badge"] is False
    assert meta["panels"]["accessories"] == [
        {"id": "hand", "kind": "gripper", "label": "Hand", "arm": "main"}
    ]
```

- [ ] **Step 3: Run the new tests and verify they fail**

Run:

```bash
python -m pytest tests/test_accessories.py tests/test_api.py::test_runtime_meta_reports_capabilities_from_config -v
```

Expected: FAIL because `AccessoryController` still expects top-level `elevator` / `grippers` config and `RuntimeServer._meta_payload()` does not expose the new `controls` / `panels` schema.

- [ ] **Step 4: Refactor `core/accessories.py` around explicit accessory kinds**

Rewrite `AccessoryController` to load from `config["accessories"]` only.

The internal schema should look like:

```python
self._accessories = {
    item["id"]: {
        "kind": item["kind"],
        "label": item["label"],
        "arm": item.get("arm"),
        ...
    }
}
```

Add an `apply_keys()` method:

```python
def apply_keys(self, keys: List[str], active_arm: str) -> None:
    with self._lock:
        for accessory in self._accessories.values():
            if accessory["kind"] == "linear_axis":
                if "ArrowUp" in keys:
                    accessory["value"] = min(
                        accessory["upper"],
                        accessory["value"] + accessory["step"],
                    )
                if "ArrowDown" in keys:
                    accessory["value"] = max(
                        accessory["lower"],
                        accessory["value"] - accessory["step"],
                    )
            elif accessory["kind"] == "gripper" and accessory.get("arm") == active_arm:
                if "KeyG" in keys:
                    accessory["aperture"] = 0.0 if accessory["aperture"] > 0.5 else 1.0
```

Update `snapshot()` to return normalized per-accessory data:

```python
{
    "column": {
        "kind": "linear_axis",
        "label": "Elevation",
        "value": -0.3,
        "unit": "m",
    },
    "right_gripper": {
        "kind": "gripper",
        "label": "Right Grip",
        "arm": "right",
        "aperture": 1.0,
        "is_closed": True,
    },
}
```

Also add a descriptor method used by `/api/meta` and the frontend:

```python
def describe(self) -> List[Dict[str, Any]]:
    with self._lock:
        return [
            {
                "id": accessory_id,
                "kind": item["kind"],
                "label": item["label"],
                "arm": item.get("arm"),
            }
            for accessory_id, item in self._accessories.items()
        ]
```

- [ ] **Step 5: Migrate `configs/tr4.yaml` to the new schema**

Replace the old `elevator` and `grippers` sections with:

```json
"accessories": [
  {
    "id": "column",
    "kind": "linear_axis",
    "label": "Elevation",
    "joint_name": "elevation",
    "lower": -0.8,
    "upper": 0.0,
    "init": -0.3,
    "step": 0.005
  },
  {
    "id": "left_gripper",
    "kind": "gripper",
    "label": "Left Grip",
    "arm": "left",
    "joint_names": [
      "left_gripper_l_joint1", "left_gripper_l_joint2", "left_gripper_l_joint3",
      "left_gripper_r_joint1", "left_gripper_r_joint2", "left_gripper_r_joint3"
    ],
    "open_positions": [-0.71, 0.70, -0.71, 0.71, -0.70, 0.71],
    "closed_positions": [-0.05, 0.00, -0.05, 0.05, 0.00, 0.05]
  },
  {
    "id": "right_gripper",
    "kind": "gripper",
    "label": "Right Grip",
    "arm": "right",
    "joint_names": [
      "right_gripper_l_joint1", "right_gripper_l_joint2", "right_gripper_l_joint3",
      "right_gripper_r_joint1", "right_gripper_r_joint2", "right_gripper_r_joint3"
    ],
    "open_positions": [-0.71, 0.70, -0.71, 0.71, -0.70, 0.71],
    "closed_positions": [-0.05, 0.00, -0.05, 0.05, 0.00, 0.05]
  }
],
```

- [ ] **Step 6: Update `renderer/viewer.py` to expose capability metadata**

In `_meta_payload()`, return:

```python
return {
    "urdf_url": f"{self.urdf_url}?v={self._urdf_version}",
    "robot_name": robot.name,
    "robot_label": str(self.config.get("robot_label", robot.name)),
    "base_link": robot.base_link,
    "ee_link": robot.ee_link,
    "joint_names": robot.joint_names,
    "joint_limits": joint_limits,
    "q_init": q_init,
    "active_arm": self.active_arm,
    "available_arms": sorted(self.config.get("arms", {}).keys()),
    "refresh_interval_ms": self.refresh_interval_ms,
    "controls": {
        "can_switch_arm": len(self.arms_data) > 1,
        "show_linear_axis_controls": any(
            item["kind"] == "linear_axis"
            for item in self.accessory.describe()
        ),
        "gripper_key": "KeyG",
    },
    "panels": {
        "show_arm_badge": len(self.arms_data) > 1,
        "accessories": self.accessory.describe(),
    },
}
```

In `/api/keyboard`, replace the hard-coded accessory branching with:

```python
server_ref.accessory.apply_keys(keys, active_arm=server_ref.active_arm)
return _send_json(
    self,
    server_ref._with_all_joints(server_ref.keyboard.apply_keys(keys, server_ref.api)),
)
```

- [ ] **Step 7: Re-run backend tests**

Run:

```bash
python -m pytest tests/test_accessories.py tests/test_api.py -v
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add core/accessories.py configs/tr4.yaml renderer/viewer.py tests/test_accessories.py tests/test_api.py
git commit -m "refactor: make accessories and meta capability-driven"
```

---

### Task 4: Make the web panel render from `/api/meta` instead of TR4-specific markup

**Files:**
- Create: `web/panel-model.js`
- Create: `web/panel-model.test.mjs`
- Modify: `web/index.html`
- Modify: `web/app.js`

- [ ] **Step 1: Write the failing frontend-model tests**

Create `web/panel-model.test.mjs`:

```javascript
import test from 'node:test';
import assert from 'node:assert/strict';

import { buildAccessoryRows, buildKeypadFlags } from './panel-model.js';

test('buildKeypadFlags hides switch-arm and linear-axis controls for single-arm robots', () => {
  const flags = buildKeypadFlags({
    controls: {
      can_switch_arm: false,
      show_linear_axis_controls: false,
    },
  });

  assert.equal(flags.showSwitchArm, false);
  assert.equal(flags.showLinearAxis, false);
});

test('buildAccessoryRows returns one row per accessory descriptor', () => {
  const rows = buildAccessoryRows(
    {
      panels: {
        accessories: [
          { id: 'hand', kind: 'gripper', label: 'Hand', arm: 'main' },
        ],
      },
    },
    {
      accessory: {
        hand: {
          kind: 'gripper',
          label: 'Hand',
          arm: 'main',
          is_closed: false,
        },
      },
    },
  );

  assert.deepEqual(rows, [
    { id: 'hand', term: 'Hand', value: 'open' },
  ]);
});
```

- [ ] **Step 2: Run the Node tests and verify they fail**

Run:

```bash
node --test web/panel-model.test.mjs
```

Expected: FAIL because `web/panel-model.js` does not exist yet.

- [ ] **Step 3: Implement `web/panel-model.js`**

Create `web/panel-model.js`:

```javascript
export function buildKeypadFlags(meta) {
  const controls = meta?.controls ?? {};
  return {
    showSwitchArm: controls.can_switch_arm === true,
    showLinearAxis: controls.show_linear_axis_controls === true,
  };
}

export function buildAccessoryRows(meta, state) {
  const descriptors = meta?.panels?.accessories ?? [];
  const accessoryState = state?.accessory ?? {};

  return descriptors.map(item => {
    const current = accessoryState[item.id] ?? {};
    if (item.kind === 'linear_axis') {
      return {
        id: item.id,
        term: item.label,
        value: `${Number(current.value ?? 0).toFixed(3)} m`,
      };
    }
    if (item.kind === 'gripper') {
      return {
        id: item.id,
        term: item.label,
        value: current.is_closed ? 'closed' : 'open',
      };
    }
    return {
      id: item.id,
      term: item.label,
      value: '-',
    };
  });
}
```

- [ ] **Step 4: Update `web/index.html` to provide dynamic containers**

Replace the fixed accessory card and fixed arm-switch/elevator keypad rows with containers:

```html
<div class="keypad-group" id="keypad-linear-axis" hidden>
  <p class="keypad-label">Linear Axis</p>
  <div class="keypad">
    <div class="keypad-row">
      <div class="key key-wide" data-code="ArrowUp">↑ &nbsp; up</div>
    </div>
    <div class="keypad-row">
      <div class="key key-wide" data-code="ArrowDown">↓ &nbsp; down</div>
    </div>
  </div>
</div>

<div class="keypad-group" id="keypad-switch-arm" hidden>
  <div class="keypad-row">
    <div class="key key-wide" data-code="KeyC">C &nbsp; switch arm</div>
  </div>
</div>

<section class="card" id="card-accessories">
  <h2>Accessories</h2>
  <dl class="pose" id="accessory-list"></dl>
</section>
```

- [ ] **Step 5: Update `web/app.js` to use the meta/state helpers**

Import the helper module:

```javascript
import { buildAccessoryRows, buildKeypadFlags } from '/web/panel-model.js';
```

Replace the fixed accessory DOM references with:

```javascript
const accessoryListEl = document.getElementById('accessory-list');
const linearAxisKeypadEl = document.getElementById('keypad-linear-axis');
const switchArmKeypadEl = document.getElementById('keypad-switch-arm');
```

In `applyMeta(nextMeta)`:

```javascript
const flags = buildKeypadFlags(nextMeta);
armBadgeEl.hidden = nextMeta.panels?.show_arm_badge !== true;
linearAxisKeypadEl.hidden = !flags.showLinearAxis;
switchArmKeypadEl.hidden = !flags.showSwitchArm;
```

Replace `applyAccessory(accessory)` with:

```javascript
function applyAccessory(accessory) {
  const rows = buildAccessoryRows(meta, { accessory });
  accessoryListEl.innerHTML = rows.map(row => `
    <dt>${row.term}</dt><dd>${row.value}</dd>
  `).join('');
  document.getElementById('card-accessories').hidden = rows.length === 0;
}
```

Also guard `doSwitchArm()`:

```javascript
if (!meta?.controls?.can_switch_arm) return;
```

- [ ] **Step 6: Re-run the Node tests**

Run:

```bash
node --test web/panel-model.test.mjs
```

Expected: PASS.

- [ ] **Step 7: Manual browser verification**

Run the app with TR4 config and confirm:

- arm badge is visible
- switch-arm key row is visible
- linear-axis key row is visible
- accessory card shows three rows: `Elevation`, `Left Grip`, `Right Grip`

Then run the app with FR3 config and confirm:

- arm badge is hidden
- switch-arm key row is hidden
- linear-axis key row is hidden
- accessory card shows exactly one row: `Hand`

- [ ] **Step 8: Commit**

```bash
git add web/index.html web/app.js web/panel-model.js web/panel-model.test.mjs
git commit -m "feat: render robot controls from capability metadata"
```

---

### Task 5: Add FR3 config, update loader coverage, and finish documentation

**Files:**
- Modify: `tests/test_loader.py`
- Modify: `tests/test_api.py`
- Create: `configs/fr3.yaml`
- Modify: `README.md`
- Modify: `main.py`

- [ ] **Step 1: Write the failing FR3 loader and runtime tests**

At the top of `tests/test_loader.py`, add:

```python
import json
from pathlib import Path
```

Then append to `tests/test_loader.py`:

```python
def test_urdf_load_fr3_chain() -> None:
    robot = load_robot_model(
        urdf_path="third_party/franka_description/simplesim/fr3_with_franka_hand.urdf",
        base_link="fr3_link0",
        ee_link="fr3_hand_tcp",
        expected_joint_names=[
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ],
    )

    assert robot.dof == 7
    assert robot.base_link == "fr3_link0"
    assert robot.ee_link == "fr3_hand_tcp"


def test_fr3_config_declares_single_arm_hand_accessory() -> None:
    config = json.loads(Path("configs/fr3.yaml").read_text(encoding="utf-8"))

    assert config["active_arm"] == "main"
    assert list(config["arms"].keys()) == ["main"]
    assert config["accessories"] == [
        {
            "id": "hand",
            "kind": "gripper",
            "label": "Hand",
            "arm": "main",
            "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
            "open_positions": [0.04, 0.04],
            "closed_positions": [0.0, 0.0],
        }
    ]
```

Append to `tests/test_api.py`:

```python
def test_runtime_meta_reports_arm_switch_for_multi_arm_only() -> None:
    robot = make_planar_robot()
    single_arm_server = RuntimeServer(
        api=RobotAPI(robot=robot, state=make_initial_state(robot)),
        keyboard=KeyboardInterface(),
        accessory=AccessoryController({"accessories": []}),
        static_root=Path("."),
        host="127.0.0.1",
        port=8765,
        urdf_url="/robot.urdf",
        arms_data={"main": (robot, make_initial_state(robot), [0.0, 0.0])},
        config={"arms": {"main": {"label": "Main", "base_link": "base", "ee_link": "ee", "joint_names": ["joint1", "joint2"], "q_init": [0.0, 0.0]}}},
        active_arm="main",
    )
    dual_arm_server = RuntimeServer(
        api=RobotAPI(robot=robot, state=make_initial_state(robot)),
        keyboard=KeyboardInterface(),
        accessory=AccessoryController({"accessories": []}),
        static_root=Path("."),
        host="127.0.0.1",
        port=8765,
        urdf_url="/robot.urdf",
        arms_data={
            "left": (robot, make_initial_state(robot), [0.0, 0.0]),
            "right": (robot, make_initial_state(robot), [0.0, 0.0]),
        },
        config={
            "arms": {
                "left": {"label": "Left", "base_link": "base", "ee_link": "ee", "joint_names": ["joint1", "joint2"], "q_init": [0.0, 0.0]},
                "right": {"label": "Right", "base_link": "base", "ee_link": "ee", "joint_names": ["joint1", "joint2"], "q_init": [0.0, 0.0]},
            }
        },
        active_arm="left",
    )

    assert single_arm_server._meta_payload()["controls"]["can_switch_arm"] is False
    assert dual_arm_server._meta_payload()["controls"]["can_switch_arm"] is True
```

- [ ] **Step 2: Run the focused FR3 tests and verify they fail**

Run:

```bash
python -m pytest tests/test_loader.py::test_fr3_config_declares_single_arm_hand_accessory tests/test_loader.py::test_urdf_load_fr3_chain tests/test_api.py::test_runtime_meta_reports_arm_switch_for_multi_arm_only -v
```

Expected: FAIL because `configs/fr3.yaml` does not exist yet.

- [ ] **Step 3: Add `configs/fr3.yaml`**

Create `configs/fr3.yaml`:

```json
{
  "robot_label": "Franka Research 3",
  "active_arm": "main",
  "arms": {
    "main": {
      "label": "Main Arm",
      "base_link": "fr3_link0",
      "ee_link": "fr3_hand_tcp",
      "joint_names": [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7"
      ],
      "q_init": [
        0.0,
        -0.78539816339,
        0.0,
        -2.35619449019,
        0.0,
        1.57079632679,
        0.78539816339
      ]
    }
  },
  "accessories": [
    {
      "id": "hand",
      "kind": "gripper",
      "label": "Hand",
      "arm": "main",
      "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
      "open_positions": [0.04, 0.04],
      "closed_positions": [0.0, 0.0]
    }
  ],
  "ik": {
    "damping": 0.12,
    "max_iterations": 160,
    "tol_pos": 0.0002,
    "tol_rot": 0.002,
    "max_step": 0.12,
    "min_step": 0.000001,
    "alpha": 1.0,
    "orientation_weight": 0.5
  },
  "keyboard": {
    "translation_step": 0.01,
    "rotation_step": 0.04
  },
  "server": {
    "host": "127.0.0.1",
    "port": 8765,
    "refresh_interval_ms": 40
  }
}
```

- [ ] **Step 4: Update the default docs and CLI examples**

In `README.md`, add a new robot-assets/config section:

```md
## Supported Robot Configs

| Robot | URDF | Config |
|------|------|--------|
| TR4 Pro dual-arm | `third_party/TR4_Pro/TR4_with_grippers_v2.urdf` | `configs/tr4.yaml` |
| Franka FR3 + hand | `third_party/franka_description/simplesim/fr3_with_franka_hand.urdf` | `configs/fr3.yaml` |
```

Add launch examples:

```bash
python main.py --urdf third_party/TR4_Pro/TR4_with_grippers_v2.urdf --config configs/tr4.yaml
python main.py --urdf third_party/franka_description/simplesim/fr3_with_franka_hand.urdf --config configs/fr3.yaml
```

- [ ] **Step 5: Re-run the full automated suite**

Run:

```bash
python -m pytest tests/ -v
node --test web/panel-model.test.mjs
```

Expected: all PASS.

- [ ] **Step 6: Manual end-to-end smoke checks**

Run both configurations separately:

```bash
python main.py --urdf third_party/TR4_Pro/TR4_with_grippers_v2.urdf --config configs/tr4.yaml
python main.py --urdf third_party/franka_description/simplesim/fr3_with_franka_hand.urdf --config configs/fr3.yaml
```

Confirm:

- TR4 preserves left/right arm switching and keeps each arm pose when switching.
- TR4 still moves the linear axis with `ArrowUp` / `ArrowDown`.
- FR3 starts with a single arm, no arm-switch affordance, and one hand status row.
- Both robots render all joints via `all_joints` without console errors.

- [ ] **Step 7: Commit**

```bash
git add tests/test_loader.py tests/test_api.py configs/fr3.yaml README.md main.py
git commit -m "feat: add FR3 robot config and coverage"
```
