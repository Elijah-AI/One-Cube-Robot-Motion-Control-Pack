# 腿部身体倾角与竖直升降

接口：`/Control/move_joints`，类型 `com_interfaces/action/MoveJoints`，部件 `arm_id: 4`。

| leg_motion_mode | 任务 YAML 名称 | 目标 | 保持的量 |
|---|---|---|---|
| 0（默认） | joint_targets | 原有关节位置目标，单位 rad | 保留原有四关节 MoveJ 和双关节补偿行为 |
| 1 | body_tilt | body_tilt_deg，绝对倾角，单位度，默认 0 | 执行时的 J1、J2、腰部、双臂和头部关节角 |
| 2 | vertical_relative | body_height_m，相对升降距离，单位 m | 身体 X/Y、身体朝向、腰部、双臂和头部关节角 |
| 3 | vertical_absolute | body_height_m，绝对高度，单位 m | 与模式 2 相同 |

身体采用 `body_link`，竖直方向采用 `base_link +Z`。绝对高度是 `body_link` **原点**在 `base_link` 中的 Z 坐标，不是 TCP 高度，也不是距离地面的相机测量值。升降以执行时的新鲜反馈为起点；已有身体倾角会保留。

倾角采用 `body_link +Z` 与竖直方向的有符号夹角：向 `base_link +X` 倾斜为正，反向为负。输入范围为 `[-180, 180]` 度，可达范围仍受执行时 J1/J2 位置及 URDF 的 J3 限位约束。J3 目标通过 URDF FK 求得；0°不等于直接把 J3 设成 0 rad。

新模式使用位置控制 `control_mode: 0`，不填写 `joint_name/position`，不使用腰部的 `relative_position` 或额外的 `acceleration_scale/jerk_scale`。`speed` 为 `(0,1]` 的速度比例，`timeout_ms` 必须为正。Action 的 `header.frame_id` 留空或填写 `base_link`。

## Action 示例

下列命令仅为接口用法。本次代码修改和离线验证没有向机器人发送这些命令。

J1/J2 保持执行起点，通过 J3 使身体竖直（省略角度也是 0°）：

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
  '{arm_id: 4, leg_motion_mode: 1, body_tilt_deg: 0.0, control_mode: 0, enable_collision_check: 1, speed: 0.10, timeout_ms: 30000}' --feedback
```

保持上半身姿态，竖直上升 2 cm；负距离为下降：

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
  '{arm_id: 4, leg_motion_mode: 2, body_height_m: 0.02, control_mode: 0, enable_collision_check: 1, speed: 0.10, timeout_ms: 30000}' --feedback
```

保持上半身姿态，将 `body_link` 原点移动到 `base_link` 中 Z=1.00 m：

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
  '{arm_id: 4, leg_motion_mode: 3, body_height_m: 1.00, control_mode: 0, enable_collision_check: 1, speed: 0.10, timeout_ms: 30000}' --feedback
```

相对高度 0 为无运动目标：不会使能腿部电机或下发轨迹，仍检查反馈、占用资源并等待稳定。成功结果的 `error_message` 中会报告基于实测关节 FK 得到的身体 Z 和倾角；它不代表外部仪器测量的物理精度。

## 任务 YAML

普通 `move_joints` 动作支持相同参数。字符串模式名仅适用于任务 YAML；直接 Action Goal 使用上表数值。

```yaml
- action_type: move_joints
  params:
    arm_id: 4
    leg_motion_mode: body_tilt
    body_tilt_deg: 0.0
    enable_collision_check: 1
    speed: 0.10
    timeout_ms: 30000

- action_type: move_joints
  params:
    arm_id: 4
    leg_motion_mode: vertical_relative
    body_height_m: 0.02
    enable_collision_check: 1
    speed: 0.10
    timeout_ms: 30000

- action_type: move_joints
  params:
    arm_id: 4
    leg_motion_mode: vertical_absolute
    body_height_m: 1.00
    enable_collision_check: 1
    speed: 0.10
    timeout_ms: 30000
```

本次未将这些动作加入任何已注册任务或修改现有抓取 YAML。

## 规划与执行

升降解算沿当前 IK 分支移动 J1/J2/J3，腰部保持不变。每个采样点及相邻点的关节插值中点都检查身体 X/Y 和朝向；URDF 数字精度的规划容差为位置 `1e-5 m`、朝向 `1e-4 rad`。Ruckig 生成平滑进度，采样周期为 4 ms；按完整非线性轨迹检查 URDF 速度上限和原腿部加速度/jerk 限制，必要时自动放慢。

不可达、奇异、限位超限、非法参数或不足以执行的超时会在下发前失败，不截断高度或夹紧 J3 目标。碰撞模式沿用 0/1/2；已启用的静态 COM 约束独立于几何碰撞开关。启用动态 ZMP 时，此独立腿部模式会拒绝执行；该模式目前使用静态平衡路径检查。

执行期间预约双臂、头部和腿部资源，检查完整、新鲜且无故障的测量反馈。保持的关节发生超过 `0.01 rad` 的偏移或明显运动时中止；升降时身体水平偏移超过 `0.01 m` 时中止。身体朝向沿用 `leg_body_orientation_tracking_tolerance_rad`。到位要求腿关节误差不超过 `0.01 rad`、身体朝向误差不超过 `0.01 rad`，升降模式还要求身体位置误差不超过 `0.001 m`，并在三个不同反馈序号上确认静止。Goal 被接受不是运动成功，应检查最终 Action Result。

## 构建和验证

`MoveJoints.action` 增加了字段，需要在同一安装前缀按 `com_interfaces → motion_control_pkg → task_manager` 重建所有消费者，再启动匹配的节点。仅替换其中一个二进制会造成接口 ABI 不一致。本次只完成本地实现和离线构建，没有部署或重启机器人。

发布目录不包含测试源码或测试构建目标。此功能的硬件效果须由匹配版本的运控节点和真实反馈确认。
