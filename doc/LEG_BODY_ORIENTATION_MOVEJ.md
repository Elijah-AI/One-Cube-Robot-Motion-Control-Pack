# 腿部两关节 MoveJ：保持 body 朝向

`/Control/move_joints` 的 `arm_id: 4` 支持两种输入：

| joint_name / position | 行为 |
| --- | --- |
| 原有四关节目标 | 原有腿部 MoveJ，四关节独立指定 |
| `leg_joint1`、`leg_joint2`、`leg_joint3` 中任意两个 | 指定两关节的绝对目标，自动补偿第三关节，全程保持 `body_link` 相对 `base_link` 的启动朝向 |

支持 J1+J2、J1+J3、J2+J3，名称顺序可以交换，`position` 与名称逐项对应，单位为弧度。`body_joint` 保持动作启动时的实测角度。body 的位置可以改变；本功能保持的是相对底座的旋转，不负责底座在世界坐标系中的运动或倾斜补偿。

使用同一个 Action，没有新增消息字段。两关节模式只接受 `control_mode: 0`、有限的 `0 < speed <= 1` 和正数 `timeout_ms`。`acceleration_scale`、`jerk_scale` 仍按腿部原接口要求省略或设为 0。原有四关节 MoveJ、头部及双臂路由保持原来的调用方式。

例如，下列请求指定 J1 和 J2；J3 由当前姿态自动计算：

```yaml
arm_id: 4
control_mode: 0
joint_name: [leg_joint1, leg_joint2]
position: [0.30, -0.50]
enable_collision_check: 1
speed: 0.15
timeout_ms: 15000
```

以上为绝对关节角参数示例。更换 `joint_name` 和对应的 `position` 即可指定另外两种组合。目标是否可执行由当时实测姿态、补偿关节限位及所选碰撞模式共同决定。

task_manager 的 `move_joints` 参数同样支持两关节目标：

```yaml
action_type: move_joints
params:
  arm_id: 4
  joint_name: [leg_joint2, leg_joint3]
  position: [-0.40, 0.25]
  enable_collision_check: 1
  speed: 0.15
  timeout_ms: 15000
```

规划使用当前加载 URDF 的关节安装旋转、轴方向、位置和速度限位。当前模型的前三腿关节构成同向俯仰链，因此其补偿关系接近 `ΔJ1 + ΔJ2 + ΔJ3 = 0`；实现根据模型推导轴方向，不按电机编号或电机正反向硬编码符号。对非平行腿轴的模型会拒绝请求。

四个关节共享一条一维 Ruckig 进度曲线，按 4 ms 采样，并精确加入终点。这样补偿关系覆盖整个运动过程。规划逐点检查关节限位和 body 朝向，朝向误差不得超过 `1e-4 rad`（约 0.0057°），该容差包含当前 URDF 中 `1.5708/3.1416` 安装角的舍入误差。补偿关节超限时拒绝整条动作，不截断目标。

两关节模式的速度限值是 `min(1.5, URDF velocity) × speed`，加速度和 jerk 限值沿用腿部 `5 × speed`、`10 × speed`，共享进度的限值同时考虑指定关节和补偿关节。碰撞模式 `0/1/2` 沿用现有接口；模式 2 仍要求有效外部点云。

执行使用 DriverSDK 的带采集时间反馈：四关节数据必须齐全、有限、无驱动故障，采集时间距现在不超过 100 ms，启动角速度不超过 0.02 rad/s。规划及使能后再次核对起点；起点变化超过 0.002 rad 时拒绝发送已规划轨迹。下发时四关节一组发送，调度延迟不会追赶补发成突发命令。

实测 body 朝向偏差超过 `leg_body_orientation_tracking_tolerance_rad` 时终止本次动作，默认 `0.02 rad`（约 1.15°），启动参数允许 `(0, 0.1] rad`。这是实测跟踪保护阈值，与更严格的规划轨迹容差分开。反馈过期、故障、取消或超时也会终止后续轨迹下发；取消后控制器保持最后下发的目标，不等于硬件急停。

成功必须有末条命令之后的新反馈，四关节实际到位误差均不超过 0.01 rad、角速度不超过 0.02 rad/s，且 body 朝向通过实测保护检查。Result 中提供 `planning_time`、`motion_time`、`execution_time` 和实际最大关节误差。

功能源码在 `motion_control_pkg`，目标格式校验头位于 `com_interfaces/include/com_interfaces/leg_joint_goal.hpp`。更新后一起重编译 `com_interfaces`、`motion_control_pkg`、`task_manager`。离线验证程序和构建产物保持在工作区外；本功能的代码实现和离线验证不代表机器人实机验收。
