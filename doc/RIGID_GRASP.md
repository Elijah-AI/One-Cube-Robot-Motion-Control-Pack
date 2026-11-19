# 双臂搬运的刚性抓取约束

`keep_grasp_rigid` 以两个工具 TCP 之间的刚体变换约束双手。单目标主从搬框和腰腿起身使用实测起姿，适用于已抓紧的双夹爪或双吸盘。双目标顶吸模式还要求两手同高、朝下。

## 双目标顶吸示例的工具标定

按用户提供的法兰/模型末端到吸盘 TCP 变换，单位米、四元数顺序 `[qx,qy,qz,qw]`：

```yaml
left_arm_tool: suck
left_arm_tool_tf: [0.0, 0.0, 0.167, 0.5, -0.5, 0.5, 0.5]
right_arm_tool: suck
right_arm_tool_tf: [0.0, 0.0, 0.167, 0.5, 0.5, 0.5, -0.5]
```

约束检查应用工具平移和旋转后的 TCP。双目标 MovePose（7）采用顶部吸取姿态，要求两 TCP 的 +Z 轴竖直向下、接触点同高。单目标主从 MoveP 和腰腿 MoveJ 使用动作起点实测的抓取姿态。运控从机器配置读取 `left_arm_tool_tf` 和 `right_arm_tool_tf`。

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

双目标模式填写 `left_target_pose` 和 `right_target_pose`，两个目标必须满足同一抓取关系。刚性协同使用同步位置控制。

### 单目标主从 MoveP

| `arm_id` | 主手 | 腰腿 |
|---|---|---|
| 0 / 1 | 左 / 右 | 固定起姿，仅双臂输出指令 |
| 5 / 6 | 左 / 右 | 参与 WBC，可用 `whole_body_waist_only: true` 只动腰和双臂 |

只给主手填写 `target_pose`。从手目标按 `主手目标 × 主手起姿的逆 × 从手起姿` 计算，三项均使用标定后的 TCP。主手可以平移并绕底盘竖直轴转动。若目标改变料框的起始倾角并超出容差，规划阶段拒绝动作。两臂同步执行，控制器用实测反馈检查相对位姿。纯手臂模式在动作期间预约腰腿，使其保持起始关节角。

以下任务 YAML 使用本次仿真的左主手目标，坐标来自该组实测起姿和工具标定。

```yaml
action_type: move_pose
params:
  arm_id: 0
  use_config_pose: true
  frame_id: base_link
  target_pose: [1.343116, 0.146780, 0.346681, 0.000269, 0.001934, 0.017671, 0.999842]
  bound_size: [0, 0, 0]
  keep_grasp_rigid: true
  grasp_position_tolerance_m: 0.002
  grasp_orientation_tolerance_rad: 0.005
  enable_move_line: false
  use_force_control: false
  enable_collision_check: 1
  speed: 0.2
  timeout_ms: 30000
```

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
    grasp_position_tolerance_m: 0.002
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

启动时应已抓好工件。腰腿 MoveJ 通过实际工具 TF 记录两 TCP 的相对位姿、左减右的有符号高差，以及竖直向上方向在各 TCP 坐标系中的表示。双手可以有高差，工具也可以朝上或倾斜。例如，左手初始高出 14 mm 时，运动中高差以 14 mm 为基准，默认允许偏差 2 mm。两臂、腿、头必须空闲且反馈新鲜、静止。动作执行期间占用这些资源，头保持起点姿态。位置控制和同步模式均为 `0`，`acceleration_scale`、`jerk_scale` 保持默认 `0`。

该模式保持指定的腰腿关节目标，由双臂补偿抓取几何；时间参数化可能因手臂运动能力而减慢整条轨迹。箱子可以整体平移、绕竖直轴转动，并非固定在世界坐标系。无法在关节限位、抓取容差、碰撞或超时约束下完成时，动作失败，不会悄悄关闭补偿。取消、执行错误及实测抓取超差共用双臂和腰腿的停止机制；成功结果需要实测到位和停止确认。

## 几何约束与规划

从动作启动时完整的实测关节状态计算两个吸盘位姿 `T_L0`、`T_R0`，记录：

```text
G = inverse(T_L0) * T_R0
inverse(T_L(t)) * T_R(t) = G
LEG MoveJ / 主从 MoveP: z_L(t) - z_R(t) = z_L(0) - z_R(0)
LEG MoveJ / 主从 MoveP: R_i(t)^T * up = R_i(0)^T * up，i 为左或右 TCP
双目标 WBC MovePose (7): z_L(t) = z_R(t)，两 TCP +Z 朝下
```

位置容差用于右 TCP 在左 TCP 坐标系内相对起点的三维位移误差，以及高差相对参考值的偏差。MoveJoints 默认 2 mm，MovePose 默认 1 mm。旋转容差默认 0.005 rad，用于双手相对旋转和各入口的倾斜约束。两种入口都允许工件整体平移和绕竖直轴转动。允许范围为 `[0.00001,0.01] m` 与 `[0.0001,0.05] rad`；这些是运动学误差阈值，实际机械精度需另行验证。

投影器联合求解左右 14 个手臂关节，使用含工具标定的完整正解，固定腰腿候选路径并考虑关节限位。MovePose 保留双臂目标端点；LEG MoveJ 保留腰腿端点并计算双臂补偿终点。失败不会返回半条路径，也不会退回旧的独立水平补偿。共用 TOPPRA 时间参数化后再检查全部指令点和点间线性插值，原有速度、加速度、jerk、碰撞及静态平衡校验继续执行。

刚性模式的 MoveL 以双吸盘中点和左吸盘姿态定义箱体参考系，对该参考系做直线平移与旋转插值，再由固定抓取偏置同时生成两个 TCP 目标。箱体转向时两个吸附点可能沿圆弧运动，因此不再要求两个 TCP 各走独立直线。最终复查箱体直线误差及双手刚性约束。

执行与到位等待阶段，将相同的双手约束注册到已有 `MotionSafety` 实测监控中；缺反馈、相对位姿或高度超差时使用现有故障锁存和指令拦截机制。抓取参考在每次动作起点捕获，本次没有新增真空吸附生命周期管理；多段动作应从同一箱体几何生成目标，不应在两段之间单独改动某只手。

## MuJoCo 验证

```bash
cd /home/elijah/ylf_wbc
./tools/run_mujoco_validation.sh --rigid --video
./tools/run_mujoco_validation.sh --rigid --replay --viewer --case leg_waist_175 --speed 1
```

输出目录为 `.mujoco_validation/rigid/`。独立 MuJoCo 正解检查双吸盘高度差、三维相对位移、间距漂移、相对旋转、向下法向、关节限位、175° 腰部端点与 WBC 目标。视频中显示箱子；右侧复用旧 `keep_tcp_level` 投影器，在相同腰腿轨迹上补偿初始双臂名义姿态，展示旧约束允许的抓取偏差。右侧是约束算法对照，不是另一次完整的 WBC 目标规划。

仿真使用 0.50 m 的吸附点间距。彩色吸盘示意的接触面放在实际标定 TCP，吸取方向为 TCP +Z，吸盘柄沿 -Z；吸盘和箱体尺寸仅用于显示，不是实际工装和箱子的测量值。箱体图形随左吸盘的固定抓取变换移动，右吸盘是否仍在其吸附点由独立数值检查判定；没有用刚性焊接把错误轨迹掩盖掉。

这是固定底盘的运动学与轨迹验证，没有模拟真空吸力、负载惯量、纸箱变形或接触动力学；刚性模式当前拒绝动态 ZMP 模式。验证脚本不会连接或驱动实机。

### 2026-11-09 本地验证结果

五组场景共检查 10,335 个 4 ms 指令点及 10,330 个点间中点，均通过独立 MuJoCo 正解检查。下表取指令点和中点中的最大误差；双手相对旋转最大误差为 0.00153°。

| 场景 | 最大高度差 / mm | 最大相对位置误差 / mm | 最大间距变化 / mm |
| --- | ---: | ---: | ---: |
| 腿部弯曲 | 0.00147 | 0.00149 | 0.00079 |
| 腰部 0→175° | 0.00367 | <0.00001 | <0.00001 |
| 腿部与腰部 0→175° 联动 | 0.00391 | 0.00155 | 0.00126 |
| 全身 MoveP | 0.07089 | 0.08945 | 0.05206 |
| 全身 MoveL | 0.05697 | 0.07345 | 0.05790 |

腿腰联动场景中，旧独立水平投影对照的最大高度差为 128.54 mm，相对位置误差为 129.08 mm。此对照说明“法向水平”本身不包含刚性抓取约束，不代表所有旧模式轨迹都会出现相同误差。

本地运控节点、任务管理器及 `com_interfaces` 已重新编译安装；四项 CTest、14 个实际 YAML 解析用例、Action 默认值与序列化检查通过，实测约束故障注入可触发现有故障锁存。视频（左右算法对照、2 倍速）、各场景 CSV 和报告保存在 `videos/mujoco_validation_rigid_20261109/`，没有执行实机动作。

回归入口：

```bash
ctest --test-dir build/motion_control_pkg \
  -R '^(rigid_grasp|leg_rigid_(grasp|execution)|tcp_level_(constraint|planning)|waist_leg_trajectory)$' --output-on-failure
```
