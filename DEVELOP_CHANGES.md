# Develop 分支主要改动

改动作者：Heng He <elijahhe2000@163.com>
提交维护身份：Elijah-AI <elijahhe2000@163.com>
真机验收日期：2026-07-30

## 1. `com_interfaces`

- 将旧双臂专用 Action 合并回统一的 `MovePose` 与 `MoveJoints`。
- 统一定义 `ARM_LEFT=0`、`ARM_RIGHT=1`、`ARM_BOTH=2`。
- 双臂目标使用显式左右字段，结果和反馈分别提供左右误差、进度及完成时间差。
- 增加接口契约测试，固定合并接口的字段和常量语义。

## 2. `motion_control_pkg`

- 增加双臂资源仲裁：左右单臂任务可独立占用、同时运行和分别结束。
- 双臂同步任务以原子方式占用两臂，避免部分占用和指令交错。
- 轨迹命令统一按 4 ms 周期发送，通过 SDK command gate 串行化 Marvin SDK 调用。
- 补充同步轨迹、任务仲裁、SDK 批处理、取消与失败传播测试。
- MovePose/MoveJoints 共用合并后的 Action Server，保留 `ARM_BOTH=2` 双臂入口。
- Pinocchio 模型从同一份左右臂 SDK 快照更新，避免左右 FK 状态时刻不一致。

## 3. `task_manager`

- 接入合并后的 MovePose/MoveJoints Action，并通过 `arm_action_adapter` 保持单臂与双臂配置兼容。
- 解除 left-only/right-only 的互斥模式：不同分支可分别发送左、右臂目标并行等待结果。
- Modbus TCP 任务入口使用本机 `192.168.1.123:12000`；任务触发/状态文档寄存器为 40001/50001。
- 增加配置解析、Action 适配、资源租约、FSM 与优先任务回归测试。
- 当前真机验收任务为左右臂 Joint1 从约 1.0 rad 低速移动到 0.8 rad。

## 4. `onecube_robot_description`

- 保留与当前运动控制匹配的 URDF、碰撞模型、显示配置及网格资源。
- 真机启动时由安装空间解析，不依赖旧工作区或临时部署路径。

## 5. 仓库清理与验证

- 删除误提交的 `build/`、`install/`、`log/` 和 `__pycache__/` 生成物。
- `.gitignore` 增加 ROS 2、Python、rosbag 和本地缓存规则。
- 保留源码级单元测试；测试代码属于主干质量保障，不属于运行时测试环境。
- 独立工作区完成 4 包编译，自动测试 88 项全部通过，其中 `task_manager` 46 项通过。
- 真机任务完成后触发寄存器恢复为 0，状态寄存器为 2，FSM 返回 IDLE。

完整编译、启动和任务命令见 `README.md`；真机数据见 `VALIDATION.md`。
