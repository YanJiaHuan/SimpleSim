# SimpleSim Dev Log

倒序(最新在上)。每条:日期 — 主题,具体改动列表,动机一行。手写,不自动生成。

## 2026-04-14 — UX overhaul

- `run.sh`:一键启动,自检子模块 / three / numpy;缺什么给提示命令,不自动安装。`$PYTHON` 变量统一 check 和 exec 的解释器。
- `main.py`:`--urdf` / `--config` 改为有默认值(TR4 Pro grippers);删除死代码 `--interface`;启动 banner 输出 URDF、active arm、刷新率、完整键位表。`q_init` 提取为局部变量传入 `RobotAPI`。
- `core/api.py`:新增 `_q_init` 字段;`home()` 回到初始位姿;`set_robot()` 原子替换机器人模型 + 状态,保留 `ik_config` / `rm_bridge`(同臂配置共用)。
- `renderer/viewer.py`:新端点 `POST /api/home`、`POST /api/switch_arm`(返回 `{meta, state}`);`GET /api/meta` 新增 `joint_limits`、`q_init`、`active_arm`、`available_arms`;`RuntimeServer` 持有 URDF 路径和完整 config 供切臂时重建模型。
- `web/index.html`:左侧 panel 重构为 6 张卡片:Robot Info / Keypad / EE Pose / Joints / IK / Actions。
- `web/styles.css`:暗色主题,卡片布局,keypad grid,关节进度条,IK 状态点。
- `web/app.js`:按键按下时 CSS 高亮;EE pose 分 x/y/z/roll/pitch/yaw 显示(带°);关节表实时更新含进度条;Home 按钮;Switch Arm 按钮(切换后重载 three.js 场景);`reloadRobot()` 先清理旧 mesh 再加载新的。
- `README.md`:重写,含理念、快速启动、键位表、目录骨架。
- `dev_log.md`:建立本文件。
- 动机:作者几个月后回来忘了启动命令;web 面板只有原始 EE pose 数组。先补"易用"再加功能。
