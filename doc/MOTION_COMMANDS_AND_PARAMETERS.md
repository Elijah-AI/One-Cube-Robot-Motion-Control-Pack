# ylf_wbc 运控指令与参数完整说明

更新日期：2026-10-08。范围：本仓库 `src/motion_control_pkg` 的实际 ROS 入口、`com_interfaces` 的运控 Action、对应的任务 YAML 适配和只读 WBC observer。本文与本次源码一起提交；运行机器必须使用同版本接口与二进制。源码、配置上传不代表机器人已部署、重启或完成实机验收。

## 目录

1. [指令总览与单位](#1-指令总览与单位)
2. [编译、启动与生效配置](#2-编译启动与生效配置)
3. [MoveJoints：手臂、头部、腰腿](#3-movejoints手臂头部腰腿)
4. [MovePose：MoveP、MoveL、双臂和全身](#4-moveposemovepmovel双臂和全身)
5. [ControlGripper 与 ControlSuck](#5-controlgripper-与-controlsuck)
6. [结果、反馈、超时和取消](#6-结果反馈超时和取消)
7. [碰撞与环境点云](#7-碰撞与环境点云)
8. [运控节点参数](#8-运控节点参数)
9. [COM、ZMP 与 observer 参数](#9-comzmp-与-observer-参数)
10. [任务 YAML 和 Modbus](#10-任务-yaml-和-modbus)
11. [反馈话题、排错和实现边界](#11-反馈话题排错和实现边界)
12. [接口原文与参数配置快照](#12-接口原文与参数配置快照)

## 1. 指令总览与单位

主节点由 [main_node.cpp](../src/motion_control_pkg/src/main_node.cpp) 建立四个 Action Server。名称是绝对路径，不随节点 namespace 改变。

| 指令/功能 | ROS 名称 | 接口类型 | 用途 |
|---|---|---|---|
| MoveJ | `/Control/move_joints` | `com_interfaces/action/MoveJoints` | 单臂、双臂、头部、腿/腰关节运动；身体倾角和竖直升降 |
| MoveP | `/Control/move_pose` | `com_interfaces/action/MovePose` | TCP 位姿目标，`enable_move_line=false` |
| MoveL | 同上 | 同上 | TCP 直线运动，`enable_move_line=true` |
| 夹爪 | `/Control/control_gripper` | `com_interfaces/action/ControlGripper` | 左/右/双夹爪 |
| 吸盘 | `/Control/control_suck` | `com_interfaces/action/ControlSuck` | 左/右/双吸盘 GPIO |
| 取消运动 | 上述运动 Action 的 `/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | 软件取消；工具 Action 当前拒绝取消 |
| 只读观察 | `wbc_model_node` / `wbc_model.launch.py` | 话题 | 计算 COM、支撑区域、反馈有效性，不下发运动 |

`MoveP`、`MoveL` 不是独立的 ROS Action 名。仓库存在 `HardwareCheck`、`HardwareReset`、`AdjustBasePose`、`PickAndPlace` 等接口文件，但它们不等于当前运控主节点提供相应服务；实际服务以 `ros2 action list -t` 和服务端实现为准。底盘行驶、硬件急停、通用清故障/复位不属于上述四个入口。

| `arm_id` | MoveJoints | MovePose |
|---|---|---|
| 0 `ARM_LEFT` | 左臂七关节 | 左 TCP |
| 1 `ARM_RIGHT` | 右臂七关节 | 右 TCP |
| 2 `ARM_BOTH` | 左右各七关节同步 | 左右 TCP 同步 |
| 3 `HEAD` | 头部 yaw/pitch | 当前主节点不接受 |
| 4 `LEG` | 腿三关节 + 腰部，或身体专用模式 | 旧腿部位姿路由，有专有限制，见第 4 节 |
| 5 `WHOLE_BODY_LEFT` | 不支持 | 左臂 + 腰腿 |
| 6 `WHOLE_BODY_RIGHT` | 不支持 | 右臂 + 腰腿 |
| 7 `WHOLE_BODY_BOTH` | 不支持 | 双臂 + 腰腿 |

全身动作不包含底盘移动和头部跟随。`whole_body_waist_only=true` 时，仅所选手臂和腰 J4/body_joint 参与，腿 J1/J2/J3 固定在起点。

统一单位：关节位置 rad，关节速度 rad/s，TCP/身体位置 m，超时 ms，结果耗时 s。唯一显式角度例外是 `body_tilt_deg`，单位度。四元数顺序为 `x,y,z,w`，不能用全零四元数。`speed` 是比例，不是 m/s 或角速度。SDK 内部的 mm/degree 转换由适配层完成，ROS 客户端不要重复转换。

目标是工具 TCP，工具变换是法兰/模型末端到 TCP 的变换。目标 `PoseStamped.header.frame_id` 必须明确，通常为 `base_link`；手臂/全身的其他坐标系必须存在于加载的 URDF 模型中，转换使用当前状态的模型 FK，不能仅发布一个任意 TF 就认为可用。Action 顶层 `header.frame_id` 不能代替各目标 PoseStamped 的 frame。

## 2. 编译、启动与生效配置

### 2.1 编译与接口版本

标准 colcon 工作区示例：

```bash
cd /path/to/ylf_wbc
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --executor sequential --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

ARM64 驱动及 NPU 视觉依赖应在相应平台准备。x86 运控 CMake 选择 pseudo EtherCAT 后端，但双臂 SDK 仍有实际连接逻辑，不能据此随意启动运控。只读 observer 使用下述独立入口。

`MoveJoints.action` 含身体运动新字段，须按 `com_interfaces → motion_control_pkg → task_manager` 重建并安装消费者。不同接口版本不可混用。本地逐包 CMake 安装可使用仓库根 [setup_local.bash](../setup_local.bash)，标准 colcon 安装使用 `install/setup.bash`。检查安装前缀：

```bash
ros2 pkg prefix com_interfaces
ros2 pkg prefix motion_control_pkg
ros2 pkg prefix task_manager
ros2 interface show com_interfaces/action/MoveJoints
ros2 interface show com_interfaces/action/MovePose
```

### 2.2 运控启动

以下启动会初始化驱动和双臂 SDK，须在准备好的机器人上执行，已有实例时不要重复启动。

```bash
ros2 launch motion_control_pkg bringup.launch.py \
  driver_config:=/opt/onecube/driver_sdk/configuration.xml \
  active_arm_configuration:=dual_arm \
  collision_pointcloud_topic:=/fusion/synced_robot_vision
```

节点名为 `/motion_control_ns/motion_control`。[bringup.launch.py](../src/motion_control_pkg/launch/bringup.launch.py) 的完整用户参数为：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `driver_config` | 环境变量 `DRIVER_SDK_CONFIG`，未设置时 `/opt/onecube/driver_sdk/configuration.xml` | 传给 `EtherCAT_bus_config` 的驱动 XML |
| `active_arm_configuration` | `dual_arm` | 仅此值受支持，单臂动作仍可使用 arm_id=0/1 |
| `collision_pointcloud_topic` | `/fusion/synced_robot_vision` | `SyncedRobotVision` 输入话题 |

launch 固定读取 **`/opt/onecube/motion_control/config/motion_control_config.yaml`**，随后覆盖 URDF、mesh、驱动 XML、手臂配置和点云话题。仓库的 `launch/config/motion_control_config.yaml` 是配置模板，修改它不自动修改 `/opt/onecube` 下生效文件。bringup 没有 `config_path` 或任意 WBC 开关的 launch 参数。

URDF 来自安装的 `onecube_robot_description/urdf/onecube_robot_description.urdf`；Marvin 配置来自安装的 `motion_control_pkg/config/ccs_m6_40.MvKDCfg`。双臂 SDK 连接 IP 在当前实现中固定为 `192.168.1.190`，不是 ROS 参数。

核对运行参数：

```bash
ros2 action list -t
ros2 param dump /motion_control_ns/motion_control
ros2 param get /motion_control_ns/motion_control urdf_path
ros2 param get /motion_control_ns/motion_control left_arm_tool_tf
ros2 param get /motion_control_ns/motion_control right_arm_tool_tf
ros2 param get /motion_control_ns/motion_control wbc_static_com_enabled
```

### 2.3 observer 启动

```bash
ros2 launch motion_control_pkg wbc_model.launch.py \
  config:=/path/to/ylf_wbc/src/motion_control_pkg/config/wbc_stage2_observer.yaml
```

`wbc_model.launch.py` 只有 `urdf_path` 和 `config` 两个参数，默认分别取安装模型与 `config/wbc_model.yaml`。该入口不启动运控，不使能 COM 运动保护；可用 `-DWBC_MODEL_ONLY=ON` 只编译模型/observer。

## 3. MoveJoints：手臂、头部、腰腿

### 3.1 Goal 全字段

未赋值的消息字段一般为零、false、空字符串/空数组；这不代表是有效运动请求。建议总是明确填写任务 ID、速度、超时、碰撞模式与目标。

| 字段 | 类型/单位 | 含义与限制 |
|---|---|---|
| `header` | `std_msgs/Header` | 时间戳/元信息；身体专用模式 frame 留空或为 `base_link` |
| `task_id` | string | 业务追踪 ID，客户端建议非空且唯一 |
| `arm_id` | uint8 | 0/1/2/3/4，见总表 |
| `control_mode` | uint8 | 0 位置；1 手臂关节阻抗。头/腿普通旧路由不会因此成为阻抗控制；身体专用和双关节补偿要求 0 |
| `joint_name` | string[] | 单臂完整唯一七关节；头部两关节；腿部规则见下文 |
| `position` | float64[]，rad | 与 joint_name 对应的绝对目标；头部/普通四关节腿路由按固定顺序读取位置 |
| `left_joint_name` / `right_joint_name` | string[7] | arm_id=2 的左右七关节名 |
| `left_position` / `right_position` | float64[7]，rad | arm_id=2 左右目标，替代 joint_name/position |
| `synchronization_mode` | uint8 | 双臂只支持 0 同时开始和结束；枚举 1 已定义但拒绝 |
| `enable_collision_check` | uint8 | 0 关闭；1 自碰撞；2 自碰撞+环境点云 |
| `speed` | float32 | 速度比例 `(0,1]`；应提供有限正值 |
| `acceleration_scale` / `jerk_scale` | float32 | `[0,1]`，0 表示沿用倍率 1；非零只允许 arm_id=0/1/2 |
| `timeout_ms` | int64，ms | 明确提供正数；各路由超时实现有差异，见第 6 节 |
| `leg_motion_mode` | uint8 | 0 关节目标；1 倾角；2 相对升降；3 绝对高度 |
| `body_tilt_deg` | float64，度 | 模式 1 的绝对有符号倾角，`[-180,180]`；其余模式必须为 0 |
| `body_height_m` | float64，m | 模式 2 为高度增量；模式 3 为绝对身体高度；模式 0/1 必须为 0 |

手臂名称为 `Joint1_L`…`Joint7_L` / `Joint1_R`…`Joint7_R`，服务端可按合法名称重排，拒绝缺失/重复。头部固定位置顺序 `[head_yaw_joint, head_pitch_joint]`；普通腿四关节固定顺序 `[leg_joint1, leg_joint2, leg_joint3, body_joint]`，不要依赖名称对这两种旧路由重排。

### 3.2 手臂、头、腿调用示例

本文所有运动数值仅用于解释格式，需替换为现场核实的可达目标。写入 Action 会运动，本次文档整理未执行这些指令。

```bash
# 左臂 MoveJ；右臂改 arm_id=1 和全部 _R 关节名
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: left_j, arm_id: 0, control_mode: 0,
  joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L],
  position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  speed: 0.1, acceleration_scale: 0.5, jerk_scale: 0.5,
  enable_collision_check: 1, timeout_ms: 30000}' --feedback

# 双臂同步 MoveJ
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: dual_j, arm_id: 2, control_mode: 0,
  left_joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L],
  left_position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  right_joint_name: [Joint1_R, Joint2_R, Joint3_R, Joint4_R, Joint5_R, Joint6_R, Joint7_R],
  right_position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  synchronization_mode: 0, speed: 0.1, enable_collision_check: 1,
  timeout_ms: 30000}' --feedback

# 头部 MoveJ：yaw、pitch
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: head_j, arm_id: 3, joint_name: [head_yaw_joint, head_pitch_joint],
  position: [0.0, 0.0], speed: 0.1, enable_collision_check: 1, timeout_ms: 30000}' --feedback

# 腿和腰四关节绝对目标，不能只给单个 body_joint 到底层 Action
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: leg_j, arm_id: 4,
  joint_name: [leg_joint1, leg_joint2, leg_joint3, body_joint],
  position: [0.0, 0.0, 0.0, 0.0], speed: 0.1,
  enable_collision_check: 1, timeout_ms: 30000}' --feedback
```

阻抗运动使用手臂 `control_mode: 1`，由第 8 节的 stiffness/damping、SDK 工具动力学参数决定控制设置；并非开启任意力控目标。

### 3.3 两关节补偿：保持身体朝向

`arm_id=4, leg_motion_mode=0` 时，恰好选择 `leg_joint1/2/3` 中两个不同关节，给绝对 rad 目标；系统通过第三关节补偿，保持 `body_link` 相对 `base_link` 的起始朝向，腰关节保持起点。身体位置允许变化。要求 `control_mode=0`、正超时、正速度。

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: leg_pair, arm_id: 4, joint_name: [leg_joint1, leg_joint2],
  position: [0.1, -0.1], control_mode: 0, speed: 0.1,
  enable_collision_check: 1, timeout_ms: 30000}' --feedback
```

另外两种合法名称组合是 `[leg_joint1,leg_joint3]`、`[leg_joint2,leg_joint3]`。不可达/补偿后越限会失败，不保证任意两个目标都可行。详见 [腿部定姿态 MoveJ](LEG_BODY_ORIENTATION_MOVEJ.md)。

### 3.4 身体倾角与竖直升降

这些模式使用空 `joint_name/position`、位置控制、零额外 acceleration/jerk scale。

| 模式 | 目标 | 保持的量 |
|---|---|---|
| 1 | `body_tilt_deg`：身体 +Z 朝 `base_link +X` 倾斜为正，0 为竖直 | 起点 J1/J2/腰/双臂/头部关节，解算 J3 |
| 2 | `body_height_m`：相对起点的 Z 增量，正升负降 | 身体 X/Y、朝向、腰、双臂和头部关节 |
| 3 | `body_height_m`：`body_link` 原点在 `base_link` 的绝对 Z | 同模式 2 |

绝对身体高度不是 TCP 高度，也不是相机到地面距离。倾角 0°不等于 J3=0 rad。倾角模式不要给高度；升降模式不要给倾角。启用动态 ZMP 时当前身体专用模式拒绝执行，静态 COM 和碰撞检查仍可使用。

```bash
# 身体竖直
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: upright, arm_id: 4, leg_motion_mode: 1, body_tilt_deg: 0.0,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 30000}' --feedback

# 相对上升 2 cm；下降填 -0.02
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: lift, arm_id: 4, leg_motion_mode: 2, body_height_m: 0.02,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 30000}' --feedback

# 绝对身体高度示例
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: body_z, arm_id: 4, leg_motion_mode: 3, body_height_m: 1.0,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 30000}' --feedback
```

身体专用运动预约上半身资源，要求完整、新鲜、无故障的反馈。到位要求腿关节误差 ≤0.01 rad、朝向误差 ≤0.01 rad；升降还要求身体位置误差 ≤0.001 m，并在三组不同序号反馈上确认停止。FK 到位误差不是外部计量精度。详见 [身体运动说明](LEG_BODY_MOTION.md)。

## 4. MovePose：MoveP、MoveL、双臂和全身

### 4.1 Goal 全字段

| 字段 | 类型/单位 | 含义与限制 |
|---|---|---|
| `header` / `task_id` | Header / string | 顶层追踪信息，目标坐标系在各 PoseStamped 内 |
| `arm_id` | uint8 | 0/1/2 手臂，5/6/7 全身；旧 LEG=4 见下文；HEAD=3 不支持 |
| `control_mode` | uint8 | 0 位置；1 手臂笛卡尔阻抗；全身的腰腿仍下发位置轨迹 |
| `target_pose` | PoseStamped | 单臂/单臂全身 TCP 目标，m + 四元数 |
| `bound_size` | float32[] | 空或三有限值；**实现是目标 XYZ 平移偏移，不是碰撞包络尺寸**，通常填 `[0,0,0]` |
| `via_poses` | PoseStamped[] | 单臂 MoveP 连续经过点，最多 64 个；每个带 frame；空为普通单目标 |
| `left_target_pose` / `right_target_pose` | PoseStamped | arm_id=2/7 显式左右 TCP 目标 |
| `left_bound_size` / `right_bound_size` | float32[3] | 左右目标平移偏移，通常置零 |
| `synchronization_mode` | uint8 | 双臂/全身双臂只允许 0；1 不支持 |
| `enable_collision_check` | uint8 | 0/1/2，独立于 COM/ZMP 开关 |
| `use_force_control` | bool | 当前手臂/全身实现要求 false；true 为预留并拒绝 |
| `enable_move_line` | bool | false MoveP；true MoveL，直线路径不能任意绕障 |
| `speed` | float32 | `(0,1]` 比例 |
| `acceleration_scale` / `jerk_scale` | float32 | `[0,1]`，0 沿用倍率 1；只有 arm_id=0/1/2 可非零 |
| `hand_action_type` | uint8[] | 当前手臂/全身要求空数组；灵巧手动作尚未接入 |
| `env_point_cloud` | PointCloud2 | 模式 2 可随 Goal 附带环境点云；格式见第 7 节 |
| `point_cloud_frame_id` | string | 保留字段；当前运控未读取，实际使用 env_point_cloud.header.frame_id |
| `timeout_ms` | int64，ms | 提供正数；双臂必须正数 |
| `whole_body_waist_only` | bool，默认 false | true 仅 arm_id=5/6/7 且 MoveP；冻结腿 J1/J2/J3 |

`bound_size` 会在目标转入规划坐标系后直接叠加 XYZ，连续 MoveP 的经过点也会叠加此偏移。任务层使用视觉偏移时要求 bound 为零，避免双重补偿。接口文件的“工具/负载包络”注释与当前执行语义不同，以此处及源码为准。

`via_poses` 只允许 arm_id=0/1、MoveP；不可与双臂、全身、MoveL 混用。它是连续经过路径，不保证在每个中间点停稳。

### 4.2 单臂 MoveP / MoveL

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{task_id: left_p, arm_id: 0, control_mode: 0,
  target_pose: {header: {frame_id: base_link}, pose: {
    position: {x: 0.4, y: 0.25, z: 1.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  bound_size: [0.0, 0.0, 0.0], enable_move_line: false,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 60000}' --feedback
```

右臂改 `arm_id=1` 并填写右 TCP。MoveL 改 `enable_move_line=true`。连续 MoveP 在 Goal 增加例如：

```yaml
via_poses:
  - header: {frame_id: base_link}
    pose:
      position: {x: 0.38, y: 0.25, z: 1.02}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

### 4.3 双臂与全身

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{task_id: dual_p, arm_id: 2, control_mode: 0,
  left_target_pose: {header: {frame_id: base_link}, pose: {
    position: {x: 0.4, y: 0.25, z: 1.0}, orientation: {w: 1.0}}},
  right_target_pose: {header: {frame_id: base_link}, pose: {
    position: {x: 0.4, y: -0.25, z: 1.0}, orientation: {w: 1.0}}},
  left_bound_size: [0.0, 0.0, 0.0], right_bound_size: [0.0, 0.0, 0.0],
  synchronization_mode: 0, enable_move_line: false,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 60000}' --feedback
```

- `arm_id=2`：双臂共享规划/执行进度，腰腿不参与联合 IK。
- `arm_id=7`：双臂和四个腰腿关节参与。改 `enable_move_line=true` 为全身双臂 MoveL。
- `arm_id=5/6`：沿用单臂示例的 `target_pose`，分别左/右臂加腰腿。
- 全身 MoveP 加 `whole_body_waist_only=true`：只让腰与所选手臂参与；不可配 MoveL，额外 acceleration/jerk scale 保持零。

同一 TCP 目标的可行性依赖执行起点、未参与关节、碰撞场景与平衡约束。IK 失败、轨迹下发失败和最终到位超时是不同阶段。选择 5/6/7 不自动启用 COM/ZMP。

### 4.4 旧腿部 MovePose 路由

`arm_id=4` 分发到腿部 `computeMovePTrajectory`，目标为腿链末端，代码强制目标 Y=0，并直接使用 x/z 和四元数。该路径没有实现普通手臂完整的 frame 转换、via、工具偏移或 MoveL 选择语义，不应当作通用六自由度 TCP 指令。身体倾角/升降应使用第 3.4 节的专用 MoveJoints 模式。

## 5. ControlGripper 与 ControlSuck

### 5.1 夹爪字段

| 字段 | 类型 | 用途与限制 |
|---|---|---|
| `header` | Header | 元信息 |
| `task_id` | string | 必须非空 |
| `gripper_id` | uint8 | 0 左；1 右；2 双 |
| `position` | uint8[] | `[左,右]` 设备位置指令，非 m/rad；不是通用毫米开口值 |
| `speed` | uint8[] | `[左,右]` 设备速度字节，非运动 Action 的 `(0,1]` 比例 |
| `max_torque` | uint8[] | `[左,右]` 设备力矩/力限制字节，非直接 Nm |
| `com_port` | uint8[] | `[左,右]`；所选侧必须为 2(COM1) 或 3(COM2)，1 为 CAN 通道不可用 |
| `block_until_complete` | bool | 必须 true，false 拒绝 |
| `timeout_ms` | int64 | 必须正数，等待设备物理状态完成 |

上述四个数组单侧也必须至少两个元素；建议恰好两个。uint8 可表示 0–255，实际行程与夹持方向按夹爪标定/型号解释。所选侧 `left_arm_tool/right_arm_tool` 必须配置为 `gripper`，且工具资源空闲。双侧失败必须检查整条 Action 结果，不能以串口写成功代替夹持完成。

```bash
# 默认模板右侧为 gripper；示例位置需按实际开闭方向选定
ros2 action send_goal /Control/control_gripper com_interfaces/action/ControlGripper \
'{task_id: right_gripper, gripper_id: 1,
  position: [0, 120], speed: [40, 40], max_torque: [30, 30],
  com_port: [2, 2], block_until_complete: true, timeout_ms: 10000}' --feedback
```

Result：`success`、`final_position[]`、`final_torque[]`、`error_message`。Feedback：`current_position[]`、`current_torque[]`、`current_speed[]`、`progress_rate`、`header`。左右固定索引 0/1；未选侧数据不能当作有效新测量。当前取消请求被拒绝，因为没有经过验证的夹爪物理停止命令。

### 5.2 吸盘字段

| 字段 | 类型 | 用途与限制 |
|---|---|---|
| `header` / `task_id` | Header / string | task_id 必须非空 |
| `suck_id` | uint8 | 0 左、1 右、2 双 |
| `active` | bool[] | 至少 `[左,右]`；true 开、false 关，未选侧保持 |
| `timeout_ms` | int64 | 必须正数 |

所选侧工具必须为 `suck`。左吸盘 GPIO4_B2，右吸盘 GPIO4_B0。

```bash
ros2 action send_goal /Control/control_suck com_interfaces/action/ControlSuck \
'{task_id: left_suck_on, suck_id: 0, active: [true, false], timeout_ms: 3000}' --feedback
```

关闭左侧把 `active[0]` 改 false；双侧改 `suck_id=2` 并给两侧状态。Result 为 `success/error_message`，Feedback 为 `header/progress_rate`。这是 GPIO 操作，没有吸附压力/工件在位验证；成功不等于物件已吸牢。当前取消请求被拒绝，GPIO 指令立即完成。

## 6. 结果、反馈、超时和取消

### 6.1 MoveJoints / MovePose Result

| 字段 | 含义 |
|---|---|
| `success` | 业务执行结果，结合 Action 终态 SUCCEEDED/ABORTED/CANCELED 判断 |
| `error_message` | 失败原因或成功补充信息，不应仅以字符串非空判失败 |
| `execution_time` | 总执行耗时 s |
| `timing_breakdown_valid` | true 才能解释三段耗时；false 时默认 0 不代表实测 |
| `ik_time` | IK 时间 s；MoveJoints 固定为 0 |
| `planning_time` | 轨迹规划/校验时间 s，不含 IK |
| `motion_time` | 下发和到位等待时间 s |
| `final_distance_error` | MoveJoints 关节误差 rad；MovePose TCP 距离误差 m；双臂通常取最大值 |
| `left_final_distance_error_rad` / `right_final_distance_error_rad` | 仅 MoveJoints，左右关节误差 |
| `left_final_distance_error_m` / `right_final_distance_error_m` | 仅 MovePose，左右 TCP 误差 |
| `completion_time_skew_sec` | 左右完成时间差 s |

不同旧部件路由不一定填充所有辅助字段。应只使用该路由实际写出的数据，不能把默认零误差视为高精度到位证据。

Feedback：`header`、`task_status`、`progress_rate`、`current_step_info`、`dual_task_status`、`left_progress_rate`、`right_progress_rate`。运动模块进度通常为 0–100；夹爪旧路径使用不同计算方式，不应把不同 Action 的数值混用。`dual_task_status` 枚举：0 QUEUED，1 PLANNING，2 EXECUTING，3 WAITING_FOR_COMPLETION，4 STOPPING。

### 6.2 完成、超时与资源

`Goal accepted` 仅表示接单，不能代表 IK 可达、轨迹可下发或实测到位。WBC 末点下发后仍等待参与手臂和四个腰腿关节的实测反馈：位置误差 ≤0.01 rad、实测/差分速度 ≤0.01 rad/s，手臂 `low_speed=true`，至少 5 组新反馈且连续稳定 100 ms。完成等待最多 3 s，同时受 Action 总超时限制；序号 100 ms 不前进、驱动错误、非有限值等都失败。阈值为代码常量，不能用不存在的 ROS 参数修改。

单臂旧路径 `timeout_ms<=0` 内部有 5000 ms 回退，双臂要求正超时；WBC 的正超时用于阶段检查及执行/完成等待。旧头部/普通腿路径没有统一实现严格的总超时守卫，填入 timeout 不保证所有阶段在该时限内退出。统一建议给正数，并通过客户端超时和取消处理异常，不承诺硬实时截止。

单臂、双臂、腰腿和工具分别有资源占用；双臂整体运动要求左右都空闲，全身及身体模式还会占用相应腰腿/上半身资源。任务层并行并不代表两个动作可以争用同一资源。

### 6.3 软件取消

正式客户端应保留 GoalHandle 并只取消自己的目标。下面请求取消 `/Control/move_pose` 的所有目标：

```bash
ros2 service call /Control/move_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
'{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
```

MoveJ 改服务路径为 `/Control/move_joints/_action/cancel_goal`。取消请求获准后仍需观察终态与实测停止。退出 `ros2 action send_goal` 终端不等于停止；软件取消不替代硬件急停。

## 7. 碰撞与环境点云

| 模式 | 自碰撞 | 环境点云参与规划 | COM/ZMP |
|---|---|---|---|
| 0 | 关闭几何检查 | 不使用 | 独立，由 WBC 参数决定 |
| 1 | 开启 | 不使用 | 同上 |
| 2 | 开启 | 必须有可用点云 | 同上 |

当前双臂模块启动时建立持久 `SyncedRobotVision` 订阅，后台接收并缓存 `camera_rgbd_pointcloud`。模式 0/1 不把点云用于几何检查，但订阅仍存在。模式 2 优先使用 MovePose 随 Goal 提供的非空 `env_point_cloud`，否则使用配置话题；MoveJoints 没有内嵌点云字段，使用话题来源。缓存读取在无可用帧时最多等待 5 s，取走后下次等待新帧；已有缓存直接使用，没有通用的点云年龄 ROS 参数，客户端/上游仍应确认采集时效。

```bash
ros2 topic info /fusion/synced_robot_vision --verbose
ros2 interface show com_interfaces/msg/SyncedRobotVision
ros2 topic hz /fusion/synced_robot_vision
```

该话题不是裸 `sensor_msgs/PointCloud2`。需要包装时可参考 [wrap_collision_cloud.py](examples/wrap_collision_cloud.py)。点云必须有真实 frame、合理 width/height、point_step/row_step/data 布局，以及 FLOAT32、count=1 的 x/y/z 字段；未知模型坐标系、空有效 XYZ、非法布局会失败。当前环境几何用 1 cm Octree，不是自动人体检测或实时急停扫描器。

当前运控不读取 `point_cloud_frame_id`，必须填写内层 `env_point_cloud.header.frame_id`。该字符串也不能修复实际点坐标错误：视觉输出在相机系时应正确转换；不要仅重命名成 `base_link`。规划场景按动作导入和清理，后台接收新帧不等于执行中自动重新规划。更完整的点云格式和接入说明见 [MOTION_WBC_USAGE.md](MOTION_WBC_USAGE.md)。

## 8. 运控节点参数

下面为源码声明的默认值；启动时 `/opt/onecube` YAML 可覆盖。参数多数在构造时缓存，修改文件后需重新启动匹配节点；即使 ROS 参数服务允许 set，也不应假定控制器缓存热更新。WBC 参数明确为 read-only。

### 8.1 模型、驱动和工具

| 参数 | 类型 / 源码默认 | 含义 |
|---|---|---|
| `EtherCAT_bus_config` | string，空格占位 | 必须由 launch 提供有效 XML 路径 |
| `urdf_path` | string，`onecube_rotbot.urdf` 占位 | launch 覆盖为安装模型；质量/惯量/限位/碰撞几何依据 |
| `mesh_dir` | string，`package` 占位 | launch 覆盖为模型 package 父级目录 |
| `dual_arm_planning_frame` | string，`base_link` | 必须是模型已知坐标系 |
| `active_arm_configuration` | string，`dual_arm` | 仅支持双通道初始化 |
| `left_arm_tool` | string，`suck` | `gripper`、`suck` 或 `none` |
| `right_arm_tool` | string，`gripper` | 同上，控制工具 Action 路由 |
| `left_arm_tool_tf` | double[7]，`[0,0,0.1785,0,0,0,1]` | 法兰/模型末端到 TCP，[m,m,m,qx,qy,qz,qw] |
| `right_arm_tool_tf` | double[7]，同上 | 右 TCP 外参 |
| `collision_pointcloud_topic` | string，`/fusion/synced_robot_vision` | 环境云的复合消息话题 |
| `arm_acceleration_scale` | double，1.0，`(0,1]` | 手臂全局加速度倍率 |
| `arm_jerk_scale` | double，1.0，`(0,1]` | 手臂全局 jerk 倍率 |
| `leg_joint_zero` | integer[4]，全 0 | 腿/腰驱动零位，设备编码单位，不是 Action 弧度目标 |
| `leg_joint_dir` | integer[4]，全 1 | 腿/腰驱动方向，按标定配置 |
| `head_joint_zero` | integer[2]，全 0 | 头部驱动零位 |
| `head_joint_dir` | integer[2]，全 1 | 头部方向 |
| `leg_body_orientation_tracking_tolerance_rad` | double，0.02，`(0,0.1]` | 腿部身体朝向运行监控容差 rad |

手臂额外加速度/jerk 倍率不改变速度上限，Goal 的 0 按 1 处理；全局倍率与单次倍率在已有 speed 缩放后进一步限制。

模板中的 TCP 长度与源码默认不同：左 z=0.159 m，右 z=0.2205 m；应按实际工具标定，不得凭默认值覆盖现场。模板含 `left_arm_joint_zero/dir/ratio`、`right_arm_joint_zero/dir/ratio` 和 `tf_lookup_timeout_sec`，当前运控路径没有声明/读取这些键。它们是历史配置项，不应以为修改它们会改变 Marvin 校准或模型 FK。

### 8.2 阻抗参数（左右分别设置）

每个前缀均有 `_stiffness`、`_damping`、`_velocity_ratio`、`_acceleration_ratio` 四个完整参数名，共 16 项：

| 前缀 | stiffness 默认（7 值） | damping 默认（7 值） | velocity / acceleration 默认 |
|---|---|---|---|
| `left_joint_impedance` | `[5,5,5,4,3,3,2]` | `[0.3,0.3,0.3,0.2,0.2,0.2,0.2]` | 10 / 10 |
| `right_joint_impedance` | 同上 | 同上 | 10 / 10 |
| `left_cartesian_impedance` | `[10000,10000,10000,600,600,600,20]` | `[0.2,0.2,0.2,0.2,0.2,0.2,1]` | 20 / 20 |
| `right_cartesian_impedance` | 同上 | 同上 | 20 / 20 |

例如 `left_joint_impedance_stiffness`、`right_cartesian_impedance_acceleration_ratio`。stiffness 必须有限且 ≥0，damping 有限且在 `[0,1]`，两数组长度必须为 7；速度/加速度 ratio 为整数 `[1,100]`。笛卡尔也使用 SDK 定义的 7 元素结构，不能自行裁成六轴。量纲和第七维按厂商 SDK 定义，不能视为 ROS 力/力矩目标。

### 8.3 工具动力学

`left_tool_dynamics`、`right_tool_dynamics` 各 10 个 double，顺序为 `[M,MCP_X,MCP_Y,MCP_Z,I_XX,I_XY,I_XZ,I_YY,I_YZ,I_ZZ]`。MCP 是相对法兰的质心，单位 **mm**；质量和惯量使用 SDK 对应单位，必须与实际负载一致。要求所有值有限、质量 >0、惯量张量物理有效。源码默认：

```yaml
left_tool_dynamics: [1.7, -0.36326579, -174.45006830, -0.57521107,
  0.005674265845, 0.000000029998, -0.000035481733,
  0.001812184129, 0.000014284801, 0.004925221043]
right_tool_dynamics: [0.35746, -0.36386, -128.866, -0.79,
  0.000598, 0.0, 0.0, 0.000177, 0.000016, 0.000602]
```

工具几何 TF 使用 m，而本参数 MCP 使用 mm，两者不能混淆。完整模板快照见附录；校验实现见 [impedance_configuration.hpp](../src/motion_control_pkg/include/motion_control_pkg/execution/impedance_configuration.hpp)。

## 9. COM、ZMP 与 observer 参数

### 9.1 运控 WBC 启动参数

| 参数 | 默认 | 作用/生效条件 |
|---|---|---|
| `wbc_static_com_enabled` | false | 开启静态 COM 支撑约束；独立于碰撞模式 |
| `wbc_dynamic_zmp_enabled` | false | 需要静态 COM 同时开启 |
| `wbc_stationary_horizontal_contact_confirmed` | false | 开启静态 COM 必须确认静止、水平、四轮接触条件，否则启动失败 |
| `wbc_support_links` | 四轮 link，见下文 | 静态 COM 开启时声明；支撑角色 |
| `wbc_unknown_steering_joints` | 两个转向 joint，见下文 | 静态阶段允许的未知转向，计算保守区域 |
| `wbc_support_inset_m` | 0.02 | 支撑区域内缩 m |
| `wbc_steering_uncertainty_margin_enabled` | true | 加入未知转向产生的额外保守边界 |
| `wbc_com_weight` | 0.01 | 静态 COM 求解权重 |
| `wbc_differentiate_joints` | `[leg_joint1,leg_joint2,leg_joint3,body_joint]` | 允许从连续位置估计速度的关节 |
| `wbc_max_state_age_sec` | 0.15 | 最大反馈年龄 s |
| `wbc_max_state_skew_sec` | 0.06 | 各关节采集时间差上限 s |
| `wbc_joint_state_topic` | `/wbc/joint_states` | 可靠全身 JointState 输入 |
| `wbc_reference_max_duration_sec` | 60.0 | 动态模式才声明；参考轨迹时长预算，有限且 >0 |
| `wbc_reference_max_zmp_scale` | 10.0 | 动态模式才声明；ZMP 时间缩放预算，有限且 ≥1 |

四轮 link 默认是 `base_front_wheel_1_pitch_link`、`base_rear_wheel_1_pitch_link`、`base_front_wheel_2_link`、`base_rear_wheel_2_link`。未知转向 joint 默认是 `base_front_wheel_1_yaw_joint`、`base_rear_wheel_1_yaw_joint`。

除前三个开关外，其余静态参数只有静态 COM 开启才声明；查询不存在不一定是版本故障。以上参数只在启动读取。下例合并到现有完整运控配置，不可覆盖掉驱动/工具/零位参数：

```yaml
/motion_control_ns/motion_control:
  ros__parameters:
    wbc_static_com_enabled: true
    wbc_dynamic_zmp_enabled: false
    wbc_stationary_horizontal_contact_confirmed: true
    wbc_joint_state_topic: /wbc/joint_states
    wbc_max_state_age_sec: 0.15
    wbc_max_state_skew_sec: 0.06
    wbc_support_inset_m: 0.02
```

现场不满足支撑假设时不能使用该确认值。动态 ZMP 要求完整新鲜的 q/v，包含实际转向角、有效速度、无假设关节；静态 observer 的假设零转向不能充当实测反馈。动态控制是位置参考规划与跟踪约束，不是力传感器 ZMP 闭环。

完整名义支撑区域配置为 `wbc_support_inset_m: 0.0` 与 `wbc_steering_uncertainty_margin_enabled: false` **同时**设置；只改前者仍有转向补偿。对应 [wbc_full_support_motion.yaml](../src/motion_control_pkg/config/wbc_full_support_motion.yaml) 只是两项片段。该配置取消额外内缩，不包含未建模载荷或地面变化。

### 9.2 只读 wbc_model 参数

observer 不使用 `wbc_` 前缀，也不会改变运控节点的保护配置。下列全部为只读启动参数：

| 参数 | 源码默认 | 说明 |
|---|---|---|
| `urdf_path` | 空，由 launch 补充 | 模型文件 |
| `support_links` | `[]` | 支撑 link，默认配置文件填四轮 |
| `package_dirs` | `[]` | 模型资源搜索目录 |
| `max_state_age_sec` | 0.1 | 反馈年龄上限 s |
| `max_state_skew_sec` | 0.02 | 采集时间差上限 s |
| `support_inset_m` | 0.02 | 支撑内缩 m |
| `steering_uncertainty_margin_enabled` | true | 转向不确定性边界 |
| `unknown_steering_joints` | `[]` | 静态假设关节列表；stage2 配置填两转向关节 |
| `differentiate_joints` | `[]` | 可进行位置差分测速的关节 |
| `velocity_filter_tau_sec` | 0.03 | 差分速度滤波时间常数 s |
| `support_coplanarity_tolerance_m` | 0.002 | 支撑点共面容差 m |
| `publish_rate_hz` | 20.0 | 输出频率，有限且 `(0,250]` |
| `joint_state_topics` | `[/joint_states]` | 必须非空；stage2 使用 `/wbc/joint_states` |

输出 `/wbc_model/status`（JSON String）、`/wbc_model/com`（PointStamped）、`/wbc_model/support_polygon`（PolygonStamped）。状态看 `position_valid`、`velocity_valid`、`missing_joints`、`stale_joints`、`assumed_joints`、COM margin 等。过期时不正常发布 COM，支撑多边形可清空；应先查 status，避免一直等待 COM。observer 的 `zmp_valid` 固定 false，不能用它证明动态 ZMP 生效。

### 9.3 离线模型诊断工具

这两个随包安装的诊断工具不初始化 ROS/硬件，不是运动指令。

```bash
ros2 run motion_control_pkg wbc_model_audit /path/to/robot.urdf --neutral \
  base_front_wheel_1_pitch_link base_rear_wheel_1_pitch_link \
  base_front_wheel_2_link base_rear_wheel_2_link

ros2 run motion_control_pkg wbc_zmp_audit /path/to/robot.urdf \
  base_front_wheel_1_pitch_link base_rear_wheel_1_pitch_link \
  base_front_wheel_2_link base_rear_wheel_2_link < states.jsonl
```

`wbc_model_audit` 位置参数为 URDF、必须显式的 `--neutral`、至少三个支撑 link；输出模型中性姿态、质量、COM、支撑和关节限位，内缩固定 0.02 m，中性状态不是实测。

`wbc_zmp_audit` 参数为 URDF 和至少四个支撑 link，标准输入每行一个 JSON 对象。必填 `joint_names`（字符串数组）、`positions`、`velocities`、`accelerations`（按名称对应的完整模型关节数值数组，rad/rad/s/rad/s²，移动关节按模型单位）；可选 `support_inset_m` 默认 0.02、`assumed_joints` 字符串数组。其他字段拒绝，单行上限 1 MiB。输出 `prediction_valid/error`、COM、所需力/力矩、ZMP 及 margin；`contact_observed=false`，不证明现场接触成立，也不支持移动根坐标或外部载荷输入。

## 10. 任务 YAML 和 Modbus

### 10.1 YAML 适配与 ROS Goal 的差异

任务由 `task_manager` 发出同样的 Action，但 YAML 不是 ROS Goal 的逐字复制。

| YAML 字段/功能 | 说明 |
|---|---|
| `action_type: move_pose` | params 为下述位姿配置 |
| `use_config_pose` | 必填 bool；true 使用 YAML 位姿，false 使用视觉数据 |
| `frame_id` | 目标坐标系，转入 PoseStamped.header.frame_id；视觉偏移/缓存要求显式填写 |
| `target_pose` | `[x,y,z,qx,qy,qz,qw]`，不是 PoseStamped 对象；单臂分支仍解析此数组 |
| `left_target_pose/right_target_pose` | 双臂同样为七元素数组 |
| `via_poses` | 七元素数组的列表，仅配置位姿、单臂 MoveP，最多 64 |
| `bound_size/left_bound_size/right_bound_size` | 必须按相应分支提供三元素数组，视觉偏移时全零 |
| `speed` | move_pose 默认 0.5；move_joints 必填；建议显式给值 |
| `use_force_control` / `timeout_ms` | move_pose 必填，当前力控应 false |
| `whole_body_waist_only` / `enable_move_line` | 默认 false，限制同 Action |
| `hand_action_type` | 默认空，当前保留空 |
| `acceleration_scale/jerk_scale` | 默认 0，范围 `[0,1]`，限制同 Action |
| `vision_result_key` | 默认空；使用命名视觉缓存时需 use_config_pose=false 和显式 frame |
| `action_type: move_joints` | 大部分字段同 Goal；body 模式支持字符串名 |
| `leg_motion_mode` | 可写 joint_targets/body_tilt/vertical_relative/vertical_absolute 或 0/1/2/3 |
| `relative_position` | 任务层独有，默认 false；仅 LEG + 单个 body_joint，可把相对腰角扩成四关节绝对目标 |
| `action_type: gripper_ctrl` | `comport`→Goal.com_port，`block`→block_until_complete，其余数组同工具接口 |
| `action_type: control_suck` | suck_id/active/timeout_ms 同吸盘 Goal |
| `delay_ms` | 动作节点字段，不在 params 内；执行前延时，默认 0 |
| `parallel_actions` | 同一序列内多个动作并行，仍受底层资源冲突检查 |
| `action_type: move_compound` | params.sub_actions 非空，只支持 move_pose/move_joints；整体 timeout_ms 默认 30000 |

**当前普通任务解析/客户端没有把 `control_mode` 作为通用参数转发到运动 Goal**，生成消息保持位置模式 0；填写 YAML `control_mode: 1` 不能据此启用阻抗。身体模式解析会校验此键必须为 0。需要阻抗时应使用明确设置 control_mode 的直接 Action 客户端。

`enable_collision_check` 缺省为 0。旧 YAML 的 false→0，true→1；只有 true 且 `enable_point_cloud_check=true` 时兼容为 2。数值 0/1/2 优先，建议统一数值格式。

任务中的腰单关节简写由新鲜 `/wbc/joint_states` 展开为四关节目标；不能直接把 `joint_name: [body_joint]` 发给底层 MoveJoints，底层普通腿要求四个位置。`relative_position` 也不是 MoveJoints.action 字段。

### 10.2 视觉偏移

`vision_pose_offset` 用于 arm_id=0/1/5/6；双臂 2/7 使用成对的 `left_vision_pose_offset/right_vision_pose_offset`。配置要求 use_config_pose=false、显式 frame、零 bound。

| 偏移字段 | 格式/默认 | 说明 |
|---|---|---|
| `translation` | 必填 `[dx,dy,dz]` m | 平移；第三项可用受限的 `current_tcp_height` |
| `rotation` | 必填 `[qx,qy,qz,qw]` | 相对旋转，有限且非零四元数 |
| `translation_frame` | `base` 默认 / `vision` | 在基准系加偏移，或先用视觉目标旋转将局部偏移转到基准系 |
| `force_vertical_down` | false | true 要求 frame_id=base_link；构造工具竖直向下姿态 |
| `minimum_z` | 默认 0 | 仅 translation[2]=current_tcp_height 时允许，非负 |

`current_tcp_height` 仅单臂 0/1、`vision_pose_offset`、frame=base_link、translation_frame=vision；要求 TCP 和腰腿反馈年龄不超过 250 ms，保持当前 TCP 的基准系高度，不是固定视觉局部 dz。

视觉动作 `vision_detect` 可用 `result_key` 缓存，`require_single_target/require_expected_frame` 验证输入。`canonical_y_direction` 为目标坐标系内非零三向量，`canonical_y_min_alignment` 默认 0.5 且在 `(0,1]`；仅在单目标、预期 frame 均检查时允许。半转规范化在缓存前进行，不应在下游再次叠加旧标定补偿。

恢复参数 `ik_failure_recovery.enabled` 默认 true、`key` 必须非空、`lift_m` 默认 0.05 且 >0、`speed` 默认 0.10 且 `(0,1]`、`timeout_ms` 默认 30000 且 >0、`enable_collision_check` 默认沿用原动作。只允许腰部参与的全身 MovePose，不能同时启用 feedback_check；`if_recovery_applied` 是恢复键，只允许放在负高度的相对升降 move_joints 上。

IK 恢复使用动作级 `ik_failure_recovery` 与 `if_recovery_applied`，详细约束见 [LAPTOP_IK_LEG_RECOVERY.md](LAPTOP_IK_LEG_RECOVERY.md)；只有匹配的 IK 失败才进入恢复，不能把反馈超时当成 IK 不可达。

### 10.3 YAML 示例

```yaml
- action_type: move_pose
  params:
    arm_id: 5
    use_config_pose: true
    frame_id: base_link
    target_pose: [0.4, 0.25, 1.0, 0.0, 0.0, 0.0, 1.0]
    bound_size: [0.0, 0.0, 0.0]
    use_force_control: false
    enable_move_line: false
    whole_body_waist_only: true
    enable_collision_check: 1
    speed: 0.1
    timeout_ms: 60000
- action_type: move_joints
  params:
    arm_id: 4
    leg_motion_mode: vertical_relative
    body_height_m: 0.02
    enable_collision_check: 1
    speed: 0.1
    timeout_ms: 30000
```

仅为动作列表片段；完整任务还需 action_sequencenum / action_sequence_N 包装及主配置注册。任务名、触发/状态寄存器、config_file 在 [task_config.yaml](../src/task_manager/config/task_config.yaml)。

### 10.4 Modbus 入口

```bash
ros2 launch task_manager task_manager.launch.py \
  config_path:=/path/to/ylf_wbc/src/task_manager/config/task_config.yaml
```

当前仓库主配置：`node_id=1`，`slave_id=1`，`listen_ip=127.0.0.1`，`listen_port=12000`，因此在任务节点所在主机本地访问。不能沿用旧文档中的固定远程 IP。`laptop_assembly_two_piece` 为 40010/50010；测试任务 40011/50011 已从本次发布配置移除。

文档寄存器号减 40000 为 Modbus 原始零基地址：40010→10，50010→10010。mbpoll 明确加 `-0`，避免默认一基寻址混淆。

```bash
# 只读查询任务状态：0 空闲、1 执行中、2 成功、3 失败
mbpoll 127.0.0.1 -p 12000 -a 1 -t 4 -0 -r 10010 -c 1 -1

# 触发笔记本任务，会实际执行整个任务流程
mbpoll 127.0.0.1 -p 12000 -a 1 -t 4 -0 -r 10 -1 1
```

触发后应读取状态和日志，不以寄存器写入成功代替动作完成。其余任务同样按实际主配置转换地址。任务前置条件、分支、重试及其他非运控任务接口见 [TASK_NODE_COMMANDS.md](TASK_NODE_COMMANDS.md)。

## 11. 反馈话题、排错和实现边界

| 话题 | 数据/用途 |
|---|---|
| `/Control/MotorInfo` | `com_interfaces/msg/MotorStatus`，手臂/工具 TCP 等状态；不要误用同名 MotorInfo.msg 的类型 |
| `/Control/HeadPose` | `com_interfaces/msg/MotorStatus`，头部相关状态 |
| `/wbc/joint_states` | `sensor_msgs/msg/JointState`，双臂 14 + 腰腿 4 + 头 2 的实测反馈 |
| `/head_joint_states` | 头部两个关节 |
| `/leg_joint_states` | 腰腿四关节 |
| `/SafeNode/heartbeat` / `/SafeNode/status` | HeartBeat / NodeStatus，节点健康信息 |

```bash
ros2 topic info /wbc/joint_states --verbose
ros2 topic echo /wbc/joint_states --once
ros2 topic hz /wbc/joint_states
ros2 topic echo /wbc_model/status --once
ros2 topic echo /Control/MotorInfo --once
```

20 关节实测反馈不包含两转向关节；动态 ZMP 的完整状态要求不能仅凭该话题名满足。不能把命令回显、失败时填零的消息或不前进的缓存当新实测值。

| 现象 | 检查/解释 |
|---|---|
| Action 不存在或字段不匹配 | 安装前缀、同版本 com_interfaces、节点启动、DDS domain |
| Goal rejected | frame、四元数、speed、同步模式、工具类型、参数组合和资源占用 |
| IK failed / Cartesian substep | 当前状态与目标、关节冻结、碰撞对、COM 和求解预算 |
| 外部点云等待失败 | SyncedRobotVision 类型、配置话题、QoS、持续有效发布，当前最多等待 5 s |
| whole-body completion timed out | 已经进入末点到位阶段；看误差、速度、low_speed 和反馈序号，非 IK 阶段 |
| motor enable rejected / drive fault | 驱动状态与真实反馈，修改几何不能消除电机故障 |
| 工具拒绝或不动作 | 对应 tool 类型、com_port、两元素数组、串口设备状态 |
| 改 YAML 没效果 | 实际加载路径、安装文件、启动缓存、是否已重启对应进程 |
| 同一目标上次可达本次失败 | 比较起始关节、碰撞云、未参与关节和约束；不能只比较终点 |

当前 WBC IK 的迭代/权重预算是源码常量，详见 [WBC_IK_TUNING.md](WBC_IK_TUNING.md)；没有可任意 ros2 param set 的同名调参接口。规划成功、Goal 接受、上传源码、构建成功、部署安装、运行新二进制、实机到位分别需要各自证据。

## 12. 接口原文与参数配置快照

以下保留完整字段类型、枚举、Result 和 Feedback，便于客户端开发核对。`---` 分隔 Goal、Result、Feedback。接口注释若与执行差异冲突（特别是 bound_size），以正文及服务端代码为准。

### MoveJoints.action

来源：[MoveJoints.action](../src/com_interfaces/action/MoveJoints.action)

```text
# 单臂和双臂共用同一个 MoveJoints Action。
uint8 ARM_LEFT=0
uint8 ARM_RIGHT=1
uint8 ARM_BOTH=2
uint8 HEAD=3
uint8 LEG=4

uint8 SYNC_START_AND_FINISH=0
uint8 SYNC_START_ONLY=1

# 控制模式：默认位置模式；阻抗模式对应厂商 SDK 的关节阻抗。
uint8 CONTROL_POSITION=0
uint8 CONTROL_IMPEDANCE=1

std_msgs/Header header
string task_id
uint8 arm_id
uint8 control_mode

# ARM_LEFT/ARM_RIGHT 使用下列可变长字段，必须提供完整、唯一的七个关节。
# LEG: 四个关节维持原有 MoveJ；仅给 leg_joint1/2/3 中两个不同关节时，
# 自动补偿第三个关节，使 body_link 相对 base_link 的朝向在全程保持启动值。
# 两关节模式使用绝对弧度目标，body_joint 保持启动角度，允许 body 位置变化。
string[] joint_name
float64[] position

# ARM_BOTH 使用下列显式左右目标，不使用 joint_name/position。
string[7] left_joint_name
float64[7] left_position
string[7] right_joint_name
float64[7] right_position
uint8 synchronization_mode

uint8 COLLISION_DISABLED=0
uint8 COLLISION_SELF=1
uint8 COLLISION_SELF_AND_ENVIRONMENT=2
# 0: off, 1: self collision, 2: self + external point cloud
uint8 enable_collision_check
float32 speed                    # 控制速度比例，范围：(0, 1]
# Additional factors on speed-scaled acceleration/jerk; 0 keeps legacy factor 1.
# Arm goals only. Velocity limit is unchanged. Valid values: [0, 1].
float32 acceleration_scale
float32 jerk_scale
int64 timeout_ms

# LEG Cartesian body modes. Zero preserves all existing joint-target behavior.
uint8 LEG_JOINT_TARGETS=0
uint8 LEG_BODY_TILT=1
uint8 LEG_BODY_Z_RELATIVE=2
uint8 LEG_BODY_Z_ABSOLUTE=3
uint8 leg_motion_mode
# Absolute signed inclination of body_link +Z toward base_link +X, degrees.
# TILT holds measured J1/J2/body_joint; omitted angle means upright (0 degrees).
float64 body_tilt_deg
# RELATIVE: delta Z; ABSOLUTE: body_link origin Z in base_link; metres.
# Both height modes hold body_link orientation, X/Y and all upper-body joints.
float64 body_height_m
---
bool success
string error_message
float64 execution_time
bool timing_breakdown_valid           # true 时下列三个阶段耗时有效
float64 ik_time                       # IK 计算耗时（MoveJoints 固定为 0），单位：秒
float64 planning_time                 # 轨迹规划与校验耗时，单位：秒
float64 motion_time                   # 轨迹下发及到位等待耗时，单位：秒
float64 final_distance_error       # 单臂最大关节误差，ARM_BOTH 时为左右最大值
float64 left_final_distance_error_rad
float64 right_final_distance_error_rad
float64 completion_time_skew_sec
---
uint8 STATUS_QUEUED=0
uint8 STATUS_PLANNING=1
uint8 STATUS_EXECUTING=2
uint8 STATUS_WAITING_FOR_COMPLETION=3
uint8 STATUS_STOPPING=4

std_msgs/Header header
string task_status
float32 progress_rate
string current_step_info
uint8 dual_task_status
float32 left_progress_rate
float32 right_progress_rate
```

### MovePose.action

来源：[MovePose.action](../src/com_interfaces/action/MovePose.action)

```text
# 单臂和双臂共用同一个 MovePose Action。
uint8 ARM_LEFT=0
uint8 ARM_RIGHT=1
uint8 ARM_BOTH=2
uint8 HEAD=3
uint8 LEG=4
uint8 WHOLE_BODY_LEFT=5
uint8 WHOLE_BODY_RIGHT=6
uint8 WHOLE_BODY_BOTH=7

uint8 SYNC_START_AND_FINISH=0
uint8 SYNC_START_ONLY=1

# 控制模式：默认位置模式；阻抗模式对应厂商 SDK 的笛卡尔阻抗。
uint8 CONTROL_POSITION=0
uint8 CONTROL_IMPEDANCE=1

std_msgs/Header header
string task_id
uint8 arm_id
uint8 control_mode

# ARM_LEFT/ARM_RIGHT 使用 target_pose/bound_size。
# Optional continuous MoveP via TCP poses; single arm only, no intermediate stops.
# Empty preserves the existing one-target action. Each via pose supplies its frame.
geometry_msgs/PoseStamped[] via_poses
geometry_msgs/PoseStamped target_pose    # 目标 6D 位姿，位置单位：米
float32[] bound_size                     # 工具/负载包络 [x,y,z]，单位：米

# ARM_BOTH 使用下列显式左右目标，不使用 target_pose/bound_size。
geometry_msgs/PoseStamped left_target_pose
geometry_msgs/PoseStamped right_target_pose
float32[3] left_bound_size
float32[3] right_bound_size
uint8 synchronization_mode

uint8 COLLISION_DISABLED=0
uint8 COLLISION_SELF=1
uint8 COLLISION_SELF_AND_ENVIRONMENT=2
# 0: off, 1: self collision, 2: self + external point cloud
uint8 enable_collision_check
bool use_force_control                   # 开启力控
bool enable_move_line                    # 开启直线运动
float32 speed                            # 控制速度比例，范围：(0, 1]
# Additional factors on speed-scaled acceleration/jerk; 0 keeps legacy factor 1.
# Arm goals only. Velocity limit is unchanged. Valid values: [0, 1].
float32 acceleration_scale
float32 jerk_scale
uint8[] hand_action_type                 # 灵巧手动作类型，未安装时留空
sensor_msgs/PointCloud2 env_point_cloud
string point_cloud_frame_id
int64 timeout_ms                         # 操作超时时间，毫秒
# 仅用于 WHOLE_BODY_LEFT/RIGHT/BOTH 的 MoveP（enable_move_line=false）。
# true: 仅所选手臂和腰部 J4/body_joint 参与，leg_joint1/2/3 保持起始角度。
# false（默认）: 保留四个腿/躯干关节参与的原全身模式。
bool whole_body_waist_only false
---
bool success
string error_message
float64 execution_time
bool timing_breakdown_valid           # true 时下列三个阶段耗时有效
float64 ik_time                       # IK 计算耗时，单位：秒
float64 planning_time                 # 路径/轨迹规划与校验耗时（不含 IK），单位：秒
float64 motion_time                   # 轨迹下发及到位等待耗时，单位：秒
float64 final_distance_error             # 单臂 TCP 误差，ARM_BOTH 时为左右最大值
float64 left_final_distance_error_m
float64 right_final_distance_error_m
float64 completion_time_skew_sec
---
uint8 STATUS_QUEUED=0
uint8 STATUS_PLANNING=1
uint8 STATUS_EXECUTING=2
uint8 STATUS_WAITING_FOR_COMPLETION=3
uint8 STATUS_STOPPING=4

string task_status
float32 progress_rate
string current_step_info
std_msgs/Header header
uint8 dual_task_status
float32 left_progress_rate
float32 right_progress_rate
```

### ControlGripper.action

来源：[ControlGripper.action](../src/com_interfaces/action/ControlGripper.action)

```text
# Goal
# 夹爪编号由接口统一定义，避免客户端和服务端重复使用裸数字。
uint8 GRIPPER_LEFT=0
uint8 GRIPPER_RIGHT=1
uint8 GRIPPER_BOTH=2

std_msgs/Header header
string task_id
uint8 gripper_id
uint8[] position     # 改为动态数组，或在代码中确保传入长度为2
uint8[] speed
uint8[] max_torque
uint8[] com_port
bool block_until_complete
int64 timeout_ms

---
# Result
bool success
uint8[] final_position
uint8[] final_torque
string error_message

---
# Feedback
uint8[] current_position
uint8[] current_torque
uint8[] current_speed
float32 progress_rate
std_msgs/Header header
```

### ControlSuck.action

来源：[ControlSuck.action](../src/com_interfaces/action/ControlSuck.action)

```text
# Goal
# GPIO4_B2 控制左吸盘，GPIO4_B0 控制右吸盘。
uint8 SUCK_LEFT=0
uint8 SUCK_RIGHT=1
uint8 SUCK_BOTH=2

std_msgs/Header header
string task_id
uint8 suck_id        #0：左吸盘，1：右吸盘，2：双吸盘
bool[] active        #[左, 右]，true：打开，false：关闭
int64 timeout_ms

---
# Result
bool success
string error_message

---
# Feedback
std_msgs/Header header
float32 progress_rate
```

### 运控 YAML 模板完整快照

此模板不是机器人当前运行参数快照。部分历史字段未被读取，见第 8 节。

```yaml
/motion_control_ns/motion_control:
  ros__parameters:

    left_arm_joint_zero: [0, 0, 0, 0, 0, 0, 0]

    left_arm_joint_dir: [1, 1, 1, 1, 1, 1, 1]
    left_arm_joint_ratio: [101, 101, 81, 81, 101, 101, 101]

    right_arm_joint_zero: [0, 0, 0, 0, 0, 0, 0]

    right_arm_joint_dir: [1, 1, 1, 1, 1, 1, 1]
    right_arm_joint_ratio: [101, 101, 81, 81, 101, 101, 101]


    leg_joint_zero: [0, 0, 0, 0]

    leg_joint_dir: [1, 1, 1, 1]

    head_joint_zero: [0, 0]

    head_joint_dir: [1, 1]

    # Independent factors on configured acceleration and jerk. Keep speed caps.
    # Start with 1.0; per-action acceleration_scale/jerk_scale can reduce further.
    arm_acceleration_scale: 1.0
    arm_jerk_scale: 1.0

    dual_arm_planning_frame: base_link
    tf_lookup_timeout_sec: 0.2
    # 仅 enable_collision_check=2 时订阅；模式 0/1 不读取外部点云。
    collision_pointcloud_topic: "/fusion/synced_robot_vision"

    left_arm_tool: suck  #根据机器人实际配置修改
    left_arm_tool_tf: [0.0, 0.0, 0.159, 0.0, 0.0, 0.0, 1.0]   #米；[x,y,z,qx,qy,qz,qw]，从模型末端/SDK法兰到工具TCP

    right_arm_tool: gripper    #根据机器人实际配置修改
    right_arm_tool_tf: [0.0, 0.0, 0.2205, 0.0, 0.0, 0.0, 1.0]   #米；[x,y,z,qx,qy,qz,qw]，从模型末端/SDK法兰到工具TCP

    # control_mode=1 时，MoveJoints 使用关节阻抗，MovePose 使用笛卡尔阻抗。
    left_joint_impedance_stiffness: [5.0, 5.0, 5.0, 4.0, 3.0, 3.0, 2.0]
    left_joint_impedance_damping: [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
    left_joint_impedance_velocity_ratio: 10
    left_joint_impedance_acceleration_ratio: 10
    right_joint_impedance_stiffness: [5.0, 5.0, 5.0, 4.0, 3.0, 3.0, 2.0]
    right_joint_impedance_damping: [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
    right_joint_impedance_velocity_ratio: 10
    right_joint_impedance_acceleration_ratio: 10

    left_cartesian_impedance_stiffness: [10000.0, 10000.0, 10000.0, 600.0, 600.0, 600.0, 20.0]
    left_cartesian_impedance_damping: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1.0]
    left_cartesian_impedance_velocity_ratio: 20
    left_cartesian_impedance_acceleration_ratio: 20
    right_cartesian_impedance_stiffness: [10000.0, 10000.0, 10000.0, 600.0, 600.0, 600.0, 20.0]
    right_cartesian_impedance_damping: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1.0]
    right_cartesian_impedance_velocity_ratio: 20
    right_cartesian_impedance_acceleration_ratio: 20

    # SDK 顺序：[M, MCP_X, MCP_Y, MCP_Z, I_XX, I_XY, I_XZ, I_YY, I_YZ, I_ZZ]。
    left_tool_dynamics: [1.7, -0.36326579, -174.45006830, -0.57521107,
      0.005674265845, 0.000000029998, -0.000035481733,
      0.001812184129, 0.000014284801, 0.004925221043]
    right_tool_dynamics: [0.35746, -0.36386, -128.866, -0.79,
      0.000598, 0.0, 0.0, 0.000177, 0.000016, 0.000602]

    # chassis_joint_dir: [-1, 1]

    # chassis_max_accx: 1.56  #1.57对应电机端3000rpm/s

    # chassis_wheel_radius: 0.075

    # chassis_width: 0.55


# 通配符/**来为所有节点或命名空间下的节点设置通用参数
# /**:
#   ros__parameters:
#     use_sim_time: true
```
