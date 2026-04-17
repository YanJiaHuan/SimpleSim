# FR3 Support And Capability-Driven Runtime — Design Spec

## Goal

Extend SimpleSim beyond the TR4 Pro case by:

- adding offline support for `FR3 + franka_hand`
- removing all RM real-hardware bridge code and assets
- making robot controls and side-panel UI capability-driven instead of TR4-specific
- moving robot description assets into `third_party/` submodules instead of keeping TR4 in a top-level folder

The result should remain a lightweight browser-based kinematics runtime: no ROS, no physics, no mandatory vendor SDKs.

## Decisions

### Robot Description Sources

- Keep upstream `third_party/urdf-loaders` as-is.
- Remove `third_party/RM_API2` entirely.
- Add official Franka description assets as a git submodule at `third_party/franka_description`.
- Move the existing TR4 Pro description assets out of the top-level `TR4_Pro/` folder and into a git submodule under `third_party/TR4_Pro`.

Rationale:

- `franka_description` stays close to upstream and makes `FR3 + franka_hand` reproducible.
- TR4 assets should live beside other third-party robot descriptions instead of as a special top-level folder.
- The runtime should treat both robots as peer inputs loaded from `third_party/`.

### Runtime Scope

- This repository becomes pure simulation/runtime only.
- No real-hardware bridge remains in the main runtime.
- No `hardware_bridge` abstraction is added in this change.
- The existing in-repo FK + DLS IK implementation remains the only kinematics backend for now.

Rationale:

- Current FK/IK are already generic URDF-chain solvers.
- Keeping one backend avoids unnecessary interface churn while the project is being generalized to multiple robots.
- Removing RM bridge code reduces weight and removes vendor-specific assumptions from the main path.

## Configuration Model

The current config shape mixes robot structure with TR4-specific concepts such as `elevator` and left/right grippers. That is replaced by explicit semantic configuration.

### Arm Definitions

`arms` remains, but arm keys are identifiers only, not semantic names. The runtime must not infer behavior from names such as `left`, `right`, `main`, `torso`, or `elevator`.

Each arm entry defines:

- `label`: human-readable UI label
- `base_link`
- `ee_link`
- `joint_names`
- `q_init`

### Accessory Definitions

Accessories become an explicit list of semantic control modules instead of hard-coded top-level sections.

Each accessory entry defines:

- `id`: stable identifier
- `kind`: semantic type such as `linear_axis` or `gripper`
- `label`: UI label
- `arm`: optional owning arm id
- `joint_names` or `joint_name`
- motion/state parameters appropriate to the accessory kind

Examples:

- TR4 elevator becomes `kind: linear_axis`
- TR4 left/right grippers become two `kind: gripper` entries
- FR3 hand becomes one `kind: gripper` entry attached to its single arm

The runtime never infers accessory type from joint names.

### UI/Capability Metadata

The server exposes explicit capability metadata in `/api/meta`, derived from config structure rather than naming conventions.

`meta` should include:

- robot identity and URDF URL
- active arm id and available arms
- joint limits and refresh interval
- `controls`
  - whether arm switching is available
  - which keypad groups should be shown
- `panels`
  - accessory widgets to render
  - labels and display types for each widget

This is explicit UI contract data. The frontend uses it directly; it does not deduce capability from string patterns.

## Frontend Behavior

The frontend becomes schema-driven for robot-specific controls.

### What Must Become Dynamic

- robot arm badge visibility and wording
- switch-arm key/button hints
- elevator keypad group
- accessories card rows
- single-gripper vs dual-gripper display

### What Stays Shared

- 3D viewer
- EE pose card
- joint table
- IK summary
- keyboard polling loop

### FR3 Expected UI

For `FR3 + franka_hand`:

- one arm only
- no arm switching control
- no elevator control
- one gripper status row
- shared pose/joints/IK cards unchanged

### TR4 Expected UI

For TR4 Pro:

- two arms with arm switch control
- one linear-axis control/status module
- two gripper status rows, one per arm

## Backend Changes

### `core/accessories.py`

Refactor from TR4-specific controller into a semantic accessory controller that:

- loads a list/map of accessories from config
- exposes current values in a generic snapshot structure
- returns renderable joint-value overrides for all accessory joints
- supports keyboard-triggered actions only for configured accessory kinds

The controller owns behavior by `kind`, not by hard-coded field names.

### `renderer/viewer.py`

`/api/meta` must expose the explicit UI contract described above.

`/api/state` must include:

- current active arm state
- `all_joints`
- normalized accessory state keyed by accessory id

`/api/switch_arm` should only be available when more than one arm exists. A single-arm robot does not need a fake no-op switch path.

### `main.py`

- remove all `RMArmBridge` setup and status output
- load robot descriptions from `third_party/...`
- continue preloading all configured arms so arm-local state persists when switching

## Robot Assets

### FR3

Use the official Franka description repository:

- source: `frankarobotics/franka_description`
- robot: `fr3`
- end effector: `franka_hand`

The implementation should materialize a local URDF entrypoint in this repo that the current loader can consume from static files. If the upstream source is xacro-based, generate and store the resulting URDF artifact needed by SimpleSim rather than making runtime depend on ROS/xacro.

### TR4

Move the current TR4 description assets into `third_party/TR4_Pro` as a submodule and update all references accordingly.

The goal is not to redesign TR4 assets, only to relocate them and keep existing behavior.

## Config Files

Add a dedicated FR3 config file, likely `configs/fr3.yaml`, that defines:

- one arm
- FR3 arm chain
- one gripper accessory for `franka_hand`
- keyboard step sizes
- server settings

Update TR4 config to the new semantic schema while preserving current runtime behavior.

## Testing

This change should be driven by failing tests before implementation.

Minimum required coverage:

- config/meta tests for single-arm robot vs multi-arm robot
- accessory snapshot tests for generic accessory schema
- API tests confirming switch-arm capability is absent or rejected for FR3
- loader tests covering FR3 arm chain selection from its generated URDF
- regression tests confirming TR4 still reports two arms, linear axis, and two grippers

Frontend-specific behavior can be covered either by lightweight DOM tests if already practical, or by server/meta tests plus manual browser verification.

## Out Of Scope

- adding Pinocchio in this change
- adding vendor-specific SDK backends
- real hardware bridge support of any kind
- trajectory planning, collision checking, or dynamics

## Risks And Mitigations

### Risk: official Franka source is xacro-oriented

Mitigation:

- keep `franka_description` as the authoritative submodule
- store only the generated URDF artifact needed by SimpleSim in-repo if runtime cannot consume xacro directly

### Risk: frontend becomes more complex

Mitigation:

- keep UI driven by a compact explicit `meta` contract
- do not let frontend re-implement backend inference logic

### Risk: relocating TR4 assets breaks relative mesh paths

Mitigation:

- preserve internal directory layout under the new submodule path
- add loader/runtime verification for both TR4 and FR3 after relocation
