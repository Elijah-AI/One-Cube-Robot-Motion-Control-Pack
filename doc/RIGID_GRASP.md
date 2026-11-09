# 双吸盘顶部搬箱：刚性协同约束

`keep_tcp_level` 只限制每个 TCP 的倾角，允许双手间距、相对 yaw 和高度差变化，不能独立承担刚性箱子的双点搬运。新增 `keep_grasp_rigid`，为两个吸盘使用同一个刚性抓取约束。

## 本次采用的标定

按用户提供的法兰/模型末端到吸盘 TCP 变换，单位米、四元数顺序 `[qx,qy,qz,qw]`：

```yaml
left_arm_tool: suck
left_arm_tool_tf: [0.0, 0.0, 0.167, 0.5, -0.5, 0.5, 0.5]
right_arm_tool: suck
right_arm_tool_tf: [0.0, 0.0, 0.167, 0.5, 0.5, 0.5, -0.5]
```

约束检查的是应用上述**完整平移和旋转后**的吸盘 TCP。吸取姿态要求两个吸盘 TCP 的 +Z 轴竖直向下，接触点同高；不是要求模型法兰的 Z 轴向下。生产节点继续使用实际加载的 `left_arm_tool_tf` / `right_arm_tool_tf`，没有改写通用默认标定或远端配置。

## 使用方式

在双臂全身 `MovePose` 或对应 `move_pose` YAML 参数中使用：

```yaml
arm_id: 7
synchronization_mode: 0
keep_tcp_level: false
keep_grasp_rigid: true
grasp_position_tolerance_m: 0.001
grasp_orientation_tolerance_rad: 0.005
use_force_control: false
enable_move_line: false  # true 为箱体参考系 MoveL
```

保留原有左右 `target_pose`、速度和超时字段。两个目标仍是原接口约定的**吸盘 TCP 目标位姿**；目标必须同时满足当前抓取关系。不能为左手和右手指定相互矛盾的间距、高度或相对旋转。单臂、普通双臂、阻抗、不同步模式、与旧水平开关同时开启等组合会被拒绝。

新增字段默认关闭，原有 `keep_tcp_level` 的独立水平行为保留。服务端和所有发送 `MovePose` / `MoveJoints` 的客户端必须使用重新生成的同一版 `com_interfaces`；更新源码后重新编译 `com_interfaces`、`motion_control_pkg` 和 `task_manager`，再加载新安装环境。

## 独立腰腿 MoveJ，双臂自动补偿

已支持 `/Control/move_joints` 的 `arm_id: 4`。在腰腿动作上加 `keep_grasp_rigid: true`，无需指定手臂目标，双臂与腰腿使用相同采样周期及启动时刻。

任务 YAML 示例：从当前腰角转到 **175°绝对角度**，腿关节保持动作起点状态：

```yaml
- action_type: move_joints
  params:
    arm_id: 4
    joint_name: [body_joint]
    position: [3.0543261909900767]  # 弧度，绝对 175°
    relative_position: false
    keep_grasp_rigid: true
    keep_tcp_level: false
    grasp_position_tolerance_m: 0.001
    grasp_orientation_tolerance_rad: 0.005
    speed: 0.2
    timeout_ms: 60000
    enable_collision_check: 1
```

`relative_position: true` 是任务管理器的相对腰角功能，按新鲜实测腰角转换成绝对目标。直接调用 ROS Action 时 `position` 始终是绝对关节角；无需 `relative_position` 字段。

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
  '{arm_id: 4, joint_name: [body_joint], position: [3.0543261909900767], keep_grasp_rigid: true, speed: 0.2, timeout_ms: 60000, enable_collision_check: 1}' --feedback
```

其他腰腿命令使用相同开关：

| 目标 | 参数 |
| --- | --- |
| 四关节腿腰 MoveJ | `joint_name: [leg_joint1, leg_joint2, leg_joint3, body_joint]`，`position` 填四个绝对弧度值；按名称映射 |
| 两腿关节 + 第三关节自动补偿 | 指定任意两个不同的 `leg_joint1/2/3`；第三关节保持原有躯干俯仰约束，腰不动 |
| 两腿关节 + 腰转 | 在上述两个腿关节后加入 `body_joint` 和目标腰角；第三腿关节自动补偿 |
| 躯干俯仰 | `leg_motion_mode: body_tilt`、`body_tilt_deg`；不填关节目标 |
| 相对升降 | `leg_motion_mode: vertical_relative`、`body_height_m`，正值升高；不填关节目标 |
| 绝对高度 | `leg_motion_mode: vertical_absolute`、`body_height_m`；不填关节目标 |

上述模式名用于 YAML；直接 ROS Action 分别用 `leg_motion_mode: 1/2/3`。高度指 `base_link` 下既有躯干高度定义。关节目标与腰角仍受 URDF 限位约束，175°不会绕过限位。

启动前双吸盘必须已朝下、同高且处于所需抓取间距；动作从这一状态记录刚性关系，不负责自动摆好吸附姿态或开启真空。两臂、腿、头必须空闲且反馈新鲜、静止。动作执行期间占用这些资源，头保持起点姿态。位置控制和同步模式均为 `0`，`acceleration_scale`、`jerk_scale` 保持默认 `0`。

该模式保持指定的腰腿关节目标，由双臂补偿抓取几何；时间参数化可能因手臂运动能力而减慢整条轨迹。箱子可以整体平移、绕竖直轴转动，并非固定在世界坐标系。无法在关节限位、抓取容差、碰撞或超时约束下完成时，动作失败，不会悄悄关闭补偿。取消、执行错误及实测抓取超差共用双臂和腰腿的停止机制；成功结果需要实测到位和停止确认。

## 几何约束与规划

从动作启动时完整的实测关节状态计算两个吸盘位姿 `T_L0`、`T_R0`，记录：

```text
G = inverse(T_L0) * T_R0
inverse(T_L(t)) * T_R(t) = G
z_L(t) = z_R(t)
吸盘法向保持向下，箱体允许整体平移和绕竖直轴转动
```

相对变换同时约束三维位移和三维旋转，因此比“只固定双手距离”更强。位置容差同时用于相对位移及高度差，旋转容差用于相对旋转和向下吸附姿态。当前默认分别为 1 mm 和 0.005 rad。允许范围为 `[0.00001,0.01] m` 与 `[0.0001,0.05] rad`；这些是运动学误差阈值，实际机械精度需另行验证。

投影器联合求解左右 14 个手臂关节，使用含工具标定的完整正解，固定腰腿候选路径并考虑关节限位。MovePose 保留双臂目标端点；LEG MoveJ 保留腰腿端点并计算双臂补偿终点。失败不会返回半条路径，也不会退回旧的独立水平补偿。共用 TOPPRA 时间参数化后再检查全部指令点和点间线性插值，原有速度、加速度、jerk、碰撞及静态平衡校验继续执行。

刚性模式的 MoveL 以双吸盘中点和左吸盘姿态定义箱体参考系，对该参考系做直线平移与旋转插值，再由固定抓取偏置同时生成两个 TCP 目标。箱体转向时两个吸附点可能沿圆弧运动，因此不再要求两个 TCP 各走独立直线。最终复查箱体直线误差及双手刚性约束。

执行与到位等待阶段，将相同的双手约束注册到已有 `MotionSafety` 实测监控中；缺反馈、相对位姿或高度超差时使用现有故障锁存和指令拦截机制。抓取参考在每次动作起点捕获，本次没有新增真空吸附生命周期管理；多段动作应从同一箱体几何生成目标，不应在两段之间单独改动某只手。

## 离线运控回归

使用真实 URDF 与 Pinocchio，检查工具标定、双手相对位姿、腿部运动保留、点间插值，以及实测反馈超差后的故障锁存和指令拦截。测试不连接硬件。

```bash
ctest --test-dir build/motion_control_pkg \
  -R '^(rigid_grasp|leg_rigid_(grasp|execution)|tcp_level_(constraint|planning)|waist_leg_trajectory)$' --output-on-failure
```

此约束用于运动学轨迹与实测位姿保护，不包含真空吸附生命周期、负载动力学或纸箱变形模型；当前不支持动态 ZMP 模式。
