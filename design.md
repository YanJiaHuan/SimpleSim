# SimpleSim Design

## Goal

Turn URDF viewing into a minimal robot runtime for TR4 single-arm control verification:

- Keyboard -> Command -> API -> IK(DLS) -> Joint -> FK -> Render
- No ROS/Gazebo/physics
- Keep third-party code untouched (`third_party/urdf-loaders`, `third_party/RM_API2`)

## Architecture

```
main.py
core/
  robot.py    # RobotModel data
  fk.py       # FK
  ik.py       # DLS IK
  state.py    # single state
  api.py      # move_joint / move_ee / get_ee_pose
loader/
  urdf.py     # URDF -> RobotModel
interface/
  keyboard.py # keyboard -> ee delta
renderer/
  viewer.py   # local HTTP runtime + API endpoints
web/
  index.html app.js styles.css
configs/
  tr4.yaml
tests/
  test_loader.py test_fk.py test_ik.py test_api.py
```

## Layer Rules

- `loader`: parse URDF only
- `core`: all kinematics and state changes
- `api`: the only controller entry
- `interface`: command generation only
- `renderer`: display and transport only

UI is forbidden to mutate joint state directly.

## Runtime Path

1. `main.py` loads config and URDF chain.
2. `RobotAPI` owns state and invokes DLS IK.
3. Web frontend uses `third_party/urdf-loaders/javascript/src/URDFLoader.js` directly.
4. Frontend key events call `/api/keyboard`.
5. Backend applies IK result and exposes `/api/state`.

## Config

`configs/tr4.yaml` is JSON-formatted YAML, including:

- active arm (`left`/`right`)
- joint chain (`base_link`, `ee_link`, `joint_names`)
- IK parameters
- keyboard step size
- local server host/port
- optional RM API bridge parameters

## Commands

Install Python test dependency:

```bash
python -m pip install pytest
```

Install web dependency:

```bash
npm install
```

Run tests:

```bash
pytest tests/
```

Run simulator:

```bash
python main.py \
  --urdf TR4_Pro/TR4_with_grippers_v2.urdf \
  --config configs/tr4.yaml \
  --interface keyboard
```

Then open `http://127.0.0.1:8765`.

