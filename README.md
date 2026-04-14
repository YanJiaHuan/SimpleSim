# SimpleSim

现有仿真器太重,URDF viewer 又只能看不能控。SimpleSim 是一个浏览器里的最小机器人 runtime:加载 URDF → 渲染 → 键盘驱动 IK → 可选桥接真机。不要 ROS,不要物理,只验证控制逻辑。

## Quick Start

```bash
git submodule update --init --recursive
npm install
./run.sh
```

然后打开 <http://127.0.0.1:8765>。

`run.sh` 会检查子模块、`node_modules/three`、以及 Python 的 `numpy`。缺什么它会告诉你要跑的命令,不会自动安装任何东西。

## Controls

| Action | Keys |
|---|---|
| Forward / Back | `W` / `S` |
| Left / Right | `A` / `D` |
| Up / Down | `Q` / `E` |
| Roll | `J` / `U` |
| Pitch | `K` / `I` |
| Yaw | `L` / `O` |
| Reset to home | `R` |
| Switch arm | `C` |

左侧面板实时显示关节角、末端位姿、IK 状态,并提供 **Home** 和 **Switch Arm** 按钮。两只手臂独立维护姿态,切换时互不影响。

## Layout

```
core/        # RobotModel, FK (world frame), DLS IK, SimulationState, RobotAPI
loader/      # URDF → RobotModel (含 base_link 世界坐标变换)
interface/   # 键盘 → EE delta (robot frame → URDF world frame)
renderer/    # http.server + /api 端点 + 静态文件
web/         # Three.js + URDFLoader 前端
configs/     # JSON 格式配置 (arms, IK, server, keyboard, rm_api)
tests/       # pytest 单元测试
```

层级规则:`interface` 和 `renderer` 不直接修改 state,所有变更通过 `core/api.py`。

## Tests

```bash
python -m pytest tests/
```
