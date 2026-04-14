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
| Translate X | `W` / `S` |
| Translate Y | `A` / `D` |
| Translate Z | `R` / `F` |
| Yaw | `Q` / `E` |
| Pitch | `↑` / `↓` |
| Roll | `←` / `→` |

左侧面板实时显示关节角、末端位姿、IK 状态,并提供 **Home** 和 **Switch Arm** 按钮。

## Layout

```
core/        # RobotModel, FK, DLS IK, state, RobotAPI (唯一控制入口)
loader/      # URDF → RobotModel
interface/   # 键盘 → EE delta 指令
renderer/    # http.server + /api 端点 + 静态文件
web/         # three.js + URDFLoader 前端
configs/     # JSON 格式的配置 (arm, IK, server, keyboard, rm_api)
```

层级规则:`interface` 和 `renderer` 不直接修改 state,所有变更通过 `core/api.py`。

## Tests

```bash
python -m pytest tests/
```
