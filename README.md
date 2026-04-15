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
| Elevator up / down | `↑` / `↓` |
| Toggle gripper | `G` |
| Reset to home | `R` |
| Switch arm | `C` |

## Panel

左侧面板各卡片说明:

| 卡片 | 内容 |
|---|---|
| Robot | 机器人名称、当前运动链 (base → ee)、激活手臂 |
| Keypad | 按键高亮,按下时点亮 |
| EE Pose | 末端位姿 (x y z roll pitch yaw) |
| Joints | 当前手臂各关节角度 + 进度条 |
| IK | 最近一次 IK 迭代数、位置误差、姿态误差 |
| Accessories | 升降高度 (m)、左右夹爪开合状态 |

两只手臂独立维护姿态,切换时互不影响。

## Config

`configs/tr4.yaml` 控制所有运行参数:

| 字段 | 说明 |
|---|---|
| `arms` | 各手臂的 base/ee link、关节名、初始角度 |
| `ik` | DLS 阻尼、迭代上限、容差、步长 |
| `elevator` | 关节名、行程范围、初始高度、步长 |
| `grippers` | 各手臂夹爪关节名、打开/关闭位置向量 |
| `keyboard` | 平移步长、旋转步长 |
| `server` | host、port、刷新间隔 |
| `rm_api` | 真机桥接参数 (disabled by default) |

夹爪 `open_positions` / `closed_positions` 直接填关节角度(rad),插值方式为线性,调参无需改代码。

## Layout

```
core/        # RobotModel, FK (world frame), DLS IK, SimulationState, RobotAPI, AccessoryController
loader/      # URDF → RobotModel (含 base_link 世界坐标变换)
interface/   # 键盘 → EE delta (robot frame → URDF world frame)
renderer/    # http.server + /api 端点 + 静态文件
web/         # Three.js + URDFLoader 前端
configs/     # JSON 格式配置
tests/       # pytest 单元测试
```

层级规则:`interface` 和 `renderer` 不直接修改 state,所有变更通过 `core/api.py`。

## Tests

```bash
python -m pytest tests/
```
