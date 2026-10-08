# ylf_wbc 运控使用命令与点云接口

最初整理于 2026-09-20；2026-10-08 更新点云实现说明。当前完整接口与参数以 [运控完整手册](MOTION_COMMANDS_AND_PARAMETERS.md) 为准。
本文命令按 ROS 2 Humble 和本工作区实际接口整理；本次只做源码与离线消息检查，没有启动硬件或执行动作。

## 1. 先区分三个入口

| 用途 | 入口 | 说明 |
|---|---|---|
| 全身运动 | `/Control/move_pose`，`com_interfaces/action/MovePose` | `arm_id=5/6/7`，所选手臂与四个腿/躯干关节联合规划、同步下发 |
| 关节运动 | `/Control/move_joints`，`com_interfaces/action/MoveJoints` | `arm_id=0/1/2/3/4`，没有 `5/6/7` 全身关节目标接口 |
| 只读模型观察 | `wbc_model.launch.py` | 消费 JointState，输出 COM、支撑区域和状态；不启动驱动，不执行动作 |

这里的全身运动是位置轨迹控制：单臂 7 + 腿/躯干 4，或双臂 14 + 腿/躯干 4。
它不自动控制底盘行驶或头部跟随，也不是一个力矩级全身控制接口。
全身 IK 可调整腿部帮助 TCP 到位，但不保证每次都会动腿。
全身 MoveP 可设置 `whole_body_waist_only: true`，仅腰部 J4 与所选手臂参与，三个腿部关节保持起始角度；默认 `false` 保留原模式。详见 [腰部与手臂 MoveP](WHOLE_BODY_WAIST_ONLY_MOVEP.md)。

`wbc_static_com_enabled`、`wbc_dynamic_zmp_enabled` 默认均为 `false`。
选择全身 `arm_id` 不等于启用了 COM/ZMP 平衡保护；碰撞开关也与这两个开关独立。

## 2. 环境、接口和启动命令

### 2.1 当前本地工作区

每个新终端执行：

```bash
cd /home/elijah/ylf_wbc
source /home/elijah/ylf_wbc/setup_local.bash
ros2 pkg prefix motion_control_pkg
ros2 pkg prefix com_interfaces
ros2 interface show com_interfaces/action/MovePose
ros2 interface show com_interfaces/action/MoveJoints
```

当前本地是逐包 CMake 独立安装，使用 `setup_local.bash`，不要假定存在根级 `install/setup.bash`。
机器人上若采用标准 colcon 安装，则加载机器人上**对应工作区**的 `install/setup.bash`。
本地 x86 构建使用离线驱动后端，不能直接当作 ARM64 真机程序。

客户端与服务器应使用同一版 `com_interfaces`，以及一致的 `ROS_DOMAIN_ID`/DDS 配置。
不混用 `ylfrobot`、`ros2_ws` 的旧 overlay。

### 2.2 真机运控启动（有硬件副作用）

仅在已部署并准备运行的机器人上使用；已有运控/EtherCAT 实例时，不重复启动：

```bash
ros2 launch motion_control_pkg bringup.launch.py \
  driver_config:=/opt/onecube/driver_sdk/configuration.xml \
  active_arm_configuration:=dual_arm \
  collision_pointcloud_topic:=/fusion/synced_robot_vision
```

节点名为 `/motion_control_ns/motion_control`，Action 名是绝对路径 `/Control/...`。
目前 `active_arm_configuration` 只支持 `dual_arm`；这不妨碍通过 Action 只动一只手臂。

**launch 实际读取 `/opt/onecube/motion_control/config/motion_control_config.yaml`。**
修改 `src/motion_control_pkg/launch/config/motion_control_config.yaml` 不会自动改变运行参数。
当前 bringup 没有 `config_path` 或 WBC 开关的 launch 参数；不要随意添加 `wbc_static_com_enabled:=true` 并假定生效。

已运行节点的只读检查：

```bash
ros2 action list -t
ros2 action info /Control/move_pose
ros2 param get /motion_control_ns/motion_control dual_arm_planning_frame
ros2 param get /motion_control_ns/motion_control left_arm_tool_tf
ros2 param get /motion_control_ns/motion_control right_arm_tool_tf
ros2 param get /motion_control_ns/motion_control collision_pointcloud_topic
ros2 param get /motion_control_ns/motion_control wbc_static_com_enabled
ros2 param get /motion_control_ns/motion_control wbc_dynamic_zmp_enabled
ros2 topic echo /Control/MotorInfo --once
```

`MotorInfo.end_effector_pose` 给出左右工具 TCP；先核对它的 `header.frame_id`。
它可辅助检查当前位姿，但不能替代 COM/ZMP 所需的可靠全身 JointState 采集流。

## 3. 全身 MoveP / MoveL 用例

**下列数值只是接口格式示例，不是已验证可达或安全的机器人姿态。执行前将位置与四元数替换为现场核实的 TCP 目标。**
首次实机全身验证按“反馈验收 → 只读 COM → 保护验证 → 低速小范围动作”进行。

| `arm_id` | MovePose 含义 | 目标字段 |
|---|---|---|
| 0 | 左臂 | `target_pose` |
| 1 | 右臂 | `target_pose` |
| 2 | 双臂，腿不参与联合求解 | `left_target_pose` + `right_target_pose` |
| 5 | 左臂 + 腿/躯干 | `target_pose` |
| 6 | 右臂 + 腿/躯干 | `target_pose` |
| 7 | 双臂 + 腿/躯干 | `left_target_pose` + `right_target_pose` |

`MovePose` 虽然定义了 `HEAD=3`，当前主节点不接受这个路由；头部关节动作使用 `MoveJoints`。

### 3.1 左臂带腿部协同（MoveP）

修改 [wbc_left_goal.yaml](examples/wbc_left_goal.yaml) 中的目标后发送：

```bash
cd /home/elijah/ylf_wbc
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
  "$(cat doc/examples/wbc_left_goal.yaml)" --feedback
```

等价的内联写法：

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
"{task_id: wbc_left_example, arm_id: 5, control_mode: 0,
  target_pose: {header: {frame_id: base_link}, pose: {
    position: {x: 0.40, y: 0.25, z: 1.00},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  bound_size: [0.0, 0.0, 0.0],
  enable_move_line: false, enable_collision_check: 1,
  use_force_control: false, speed: 0.10, timeout_ms: 60000}" --feedback
```

右臂带腿部协同：改为 `arm_id: 6`，并填写右 TCP 目标。
仅左/右臂运动：分别改为 `arm_id: 0/1`。

### 3.2 全身直线运动（MoveL）

仍使用 `/Control/move_pose`，将对应 YAML 中：

```yaml
enable_move_line: true
```

然后发送同一条命令。双臂模式也支持 MoveL，左右共享进度参数。
直线路径遇到不可达/碰撞时可能失败，不应期待仍保持直线又任意绕障。
`enable_move_line: false` 不承诺 TCP 沿直线运动。

### 3.3 双臂带腿部协同，同时启用点云防碰撞

先按第 4 节接入点云，修改 [wbc_both_cloud_goal.yaml](examples/wbc_both_cloud_goal.yaml) 中左右 TCP 目标，再发送：

```bash
cd /home/elijah/ylf_wbc
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
  "$(cat doc/examples/wbc_both_cloud_goal.yaml)" --feedback
```

这个用例关键字段为：

```yaml
arm_id: 7
synchronization_mode: 0
enable_collision_check: 2
enable_move_line: false
speed: 0.10
timeout_ms: 60000
```

模式 `2/7` 必须分别填写左右目标，不能只填 `target_pose`。
当前双臂请求只接受 `synchronization_mode: 0`，不支持 `1`。
单臂全身和双臂全身均需要正数 `timeout_ms`。

### 3.4 目标与可选字段的实际语义

- 位置单位为米，姿态为四元数 `[x,y,z,w]`，不是 RPY 或角度。
- 每个 `PoseStamped.header.frame_id` 都要填；只填最外层 `goal.header.frame_id` 不够。
- 建议目标直接使用真实 `base_link` 数值。全身目标的其他 frame 通过当前关节状态和模型 FK 转换，必须是模型已有 frame；外部 TF 中有同名 frame 并不代表能用。
- 目标是配置工具的 TCP，工具变换来自运行参数 `left_arm_tool_tf/right_arm_tool_tf`。
- **全身实现把 `bound_size/left_bound_size/right_bound_size` 直接作为规划坐标系下的 XYZ 位移加到目标上。** 虽然 Action 注释称其为工具/负载包络，它在此处不是避障包围盒或障碍膨胀半径；不用偏移时保持 `[0,0,0]`。
- `speed` 为 `(0,1]` 比例，不是 m/s。
- 全身 `acceleration_scale`、`jerk_scale` 必须为 `0` 或省略；非零会拒绝。普通手臂 `0/1/2` 才支持这两个字段，`0` 表示默认倍率 1。
- 全身不能使用 `via_poses`；它只用于单臂 `0/1` 的连续 MoveP，最多 64 个中间位姿。
- 示例使用 `control_mode: 0` 位置模式。`use_force_control: true` 和非空 `hand_action_type` 当前会被拒绝；不要用它们开启笛卡尔阻抗。

## 4. 点云防碰撞怎么接

### 4.1 三种模式

| `enable_collision_check` | 行为 |
|---|---|
| 0 | 关闭几何碰撞检查；不读取外部点云 |
| 1 | 自碰撞；不读取外部点云 |
| 2 | 自碰撞 + 环境点云；缺少有效环境点云时失败 |

该字段是 `uint8`，使用数字 `0/1/2`。以前写 `true` 的配置不能表达环境防碰撞。
这些模式不关闭已经启用的 WBC COM 约束。

### 4.2 路径 A：使用融合话题

默认输入：

```text
话题：/fusion/synced_robot_vision
类型：com_interfaces/msg/SyncedRobotVision
读取字段：camera_rgbd_pointcloud（sensor_msgs/msg/PointCloud2）
```

不是直接订阅裸 `PointCloud2`。将 `collision_pointcloud_topic` 指向普通 `/camera/points`，类型不同，收不到点云。

只读检查：

```bash
ros2 topic type /fusion/synced_robot_vision
ros2 topic info /fusion/synced_robot_vision --verbose
ros2 topic echo /fusion/synced_robot_vision \
  --field camera_rgbd_pointcloud.header --once
ros2 topic echo /fusion/synced_robot_vision \
  --field camera_rgbd_pointcloud.fields --once
ros2 topic hz /fusion/synced_robot_vision
```

接着发送 `enable_collision_check: 2` 的动作，`env_point_cloud` 保持默认空。
程序启动时建立持久订阅并后台缓存非空点云；规划读取无可用缓存时最多等待 **5 秒**，已有缓存直接使用，取走后下次等新帧。
订阅使用 `SensorDataQoS`，动作未开始时订阅也存在。发布端应持续更新；当前缓存没有通用年龄限制，仍需核对采集时效。

多相机共用融合话题时，当前碰撞读取器不按 `camera_id` 筛选，也不合并多路点云，使用最近缓存的非空一帧。
若需要固定相机或融合完整环境，应在上游生成专门的碰撞点云话题。
当前读取器也不使用外层 `data_valid`、`joint_positions` 等字段进行有效性或同步判定。

### 4.3 已有裸 PointCloud2：封装后接入

提供了只做消息封装的 [wrap_collision_cloud.py](examples/wrap_collision_cloud.py)。
输入应已经做过所需坐标转换和环境筛选；脚本保留输入 frame、stamp 和点数据，不执行 TF 转换、滤波或运动。

终端 A（将 `/environment/points_base` 替换为自己的真实环境点云话题）：

```bash
source /home/elijah/ylf_wbc/setup_local.bash
python3 /home/elijah/ylf_wbc/doc/examples/wrap_collision_cloud.py \
  --input /environment/points_base \
  --output /collision/synced_robot_vision
```

运控下次启动时选择这个专用输入：

```bash
ros2 launch motion_control_pkg bringup.launch.py \
  collision_pointcloud_topic:=/collision/synced_robot_vision
```

不要为了换话题启动第二个运控实例。该话题参数在初始化时读入成员变量，在线 `ros2 param set` 不会重新创建读取配置，需要有计划地重启运控。

### 4.4 路径 B：随 MovePose Goal 传点云

自定义 Python/C++ Action 客户端可直接填写：

```python
# goal 是已填好目标的 com_interfaces.action.MovePose.Goal()
# cloud 是 sensor_msgs.msg.PointCloud2，来自实际环境点云。
goal.enable_collision_check = 2
goal.env_point_cloud = cloud
goal.point_cloud_frame_id = cloud.header.frame_id  # 可保持一致，但不是解析依据
# 然后通过 /Control/move_pose 的 ActionClient 发送 goal。
```

`env_point_cloud.data` 非空时优先使用 Goal 内点云，不再读取话题；这份点云无效会失败，不会回退到话题。
Goal 内点云为空时才走路径 A。
**运控实际读取 `env_point_cloud.header.frame_id`，并未使用 `point_cloud_frame_id` 修复或覆盖 frame。**
不建议在终端手写大量 `data: [字节...]`，应用程序直接传 PointCloud2 更合适。
`MoveJoints` 没有 `env_point_cloud` 字段，需要模式 2 时只能走话题输入。

## 5. PointCloud2 的格式要求

| 项目 | 当前实现要求 |
|---|---|
| 单位 | XYZ 均为米；毫米需在上游换算 |
| `header.frame_id` | 非空，且必须存在于运控加载的 Pinocchio/URDF 模型中；推荐实际转换到 `base_link` |
| `width` / `height` | 都大于 0；无组织云可用 `width=N,height=1`，有组织云也可用 |
| `fields` | 必须有小写 `x`、`y`、`z`；各自 `datatype=7`（FLOAT32）、`count=1` |
| 字段 `offset` | 每个 XYZ 的 `offset+4 <= point_step` |
| `point_step` | 大于 0；紧密 XYZ 常用 12，XYZRGB 常用 16，允许其他合法布局 |
| `row_step` | `>= width * point_step` |
| `data` | 至少 `row_step * height` 字节 |
| `is_bigendian` | 必须为 `false`，当前拒绝大端数据 |
| 非有限值 | NaN/Inf 点被跳过；必须剩下至少一个有限 XYZ 点 |
| RGB/intensity | 防碰撞不要求、不使用；有这些附加字段也可以 |
| `is_dense` | 可为 false；实现仍逐点检查有限性 |
| `header.stamp` | 应保留真实采集时间；当前碰撞导入未做点云年龄阈值或按采集时刻的 TF 查找 |

最小紧密布局：

```yaml
header:
  frame_id: base_link
height: 1
width: N
fields:
  - {name: x, offset: 0, datatype: 7, count: 1}
  - {name: y, offset: 4, datatype: 7, count: 1}
  - {name: z, offset: 8, datatype: 7, count: 1}
is_bigendian: false
point_step: 12
row_step: 12*N
data: "N 个小端 float32 XYZ 的二进制字节，合计 12*N 字节"
is_dense: true
```

上面的 `N`、`12*N` 和 `data` 是结构说明，不能原样作为 ROS 消息发送。
Python 可用以下代码构造合法布局；示例点仅演示编码，不代表真实环境：

```python
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

header = Header(frame_id='base_link')
header.stamp = node.get_clock().now().to_msg()  # 实际相机数据使用采集时间
points = [(0.60, 0.20, 0.80), (0.60, 0.21, 0.80)]
cloud = point_cloud2.create_cloud_xyz32(header, points)
```

注意：

- 不能只把相机云 `frame_id` 改成 `base_link`；必须同步真正变换 XYZ。
- 当前坐标转换查询机器人模型，不查询任意外部 TF。`camera_color_optical_frame` 若不在模型中会失败；即使 TF 发布了它也不够。
- 对已变换到 `base_link` 的点云不再做第二次相机外参变换。本仓库相机节点有转换逻辑，但应检查实际输出 header 和标定是否匹配，不能只依赖消息注释。
- 碰撞层不自动滤掉机器人自身、夹具、地面或被抓物体。上游需按实际场景处理自体点与接触面；不要将整片有效障碍任意删除。
- 所有有限点都按障碍插入，包括无效深度若被错误编码成 `(0,0,0)` 的点。无效深度应剔除或使用 NaN。
- 点云被构造成固定 **0.01 m** 分辨率的占据 Octree。建议上游做合理 ROI/降采样；代码没有保证固定的最大点数或规划延时。

## 6. 防碰撞的能力边界

当前行为是“取一帧环境 → 转到模型根坐标系 → 建碰撞场景 → 求解/规划/轨迹校验”。
2026-09-20 起，点云加入场景后的碰撞对象名固定为 `box`，与 ROS 话题名、视觉目标名和 `frame_id` 无关。
已经处于模型根坐标系（本模型为 `base_link`）的有限 XYZ 点明确走直通分支，不再次乘相机外参；
其他模型 frame 仍执行一次实际坐标转换。场景退出时移除 `box`，下次动作重新建立。
它不是持续刷新点云、在线重规划和动态障碍急停的闭环。
执行期间新出现的人或障碍不一定被这次动作检测到。

点云只描述已观测的占据体素，不能把未观测区域当作已经证实安全。
`bound_size` 也不能代替工具/负载真实碰撞几何。
全身 MoveP 可以使用规划器寻找可行路径，但开启模式 2 不保证任何目标都能绕行成功。

## 7. COM 观察与启用 WBC 平衡约束

### 7.1 只读观察

默认 observer 配置消费 `/joint_states`。若现场可信采集流是 `/wbc/joint_states`，可使用仓库的静态阶段配置：

```bash
source /home/elijah/ylf_wbc/setup_local.bash
ros2 launch motion_control_pkg wbc_model.launch.py \
  config:=/home/elijah/ylf_wbc/src/motion_control_pkg/config/wbc_stage2_observer.yaml
```

另一个终端：

```bash
ros2 topic info /wbc/joint_states --verbose
ros2 topic echo /wbc/joint_states --once
ros2 topic hz /wbc/joint_states
ros2 topic echo /wbc_model/status --once
ros2 topic echo /wbc_model/com --once
ros2 topic echo /wbc_model/support_polygon --once
```

`status` 为 JSON 字符串。关注 `position_valid`、`velocity_valid`、`missing_joints`、`stale_joints`、`assumed_joints`、
`com_projection_inside_inset`、`com_margin_after_inset_m`。
没有完整新鲜状态时不会正常发布 COM，`echo /wbc_model/com --once` 可能持续等待；先看 status。
静态阶段配置允许转向关节假设并计算保守区域，因此不是“完整实测状态”或动态 ZMP 验收。
observer 的 `zmp_valid` 固定为 false；启动 observer 不会开启运控的动态 ZMP 功能。

### 7.2 运控启用静态 COM 保护

在真机实际生效的 YAML 对应节点下合并这些字段，保留原有零位、方向、工具和驱动配置：

```yaml
/motion_control_ns/motion_control:
  ros__parameters:
    wbc_static_com_enabled: true
    wbc_dynamic_zmp_enabled: false
    # 只在现场确实满足静止、水平、四轮接触条件时声明 true。
    wbc_stationary_horizontal_contact_confirmed: true
    wbc_joint_state_topic: /wbc/joint_states
    wbc_max_state_age_sec: 0.15
    wbc_max_state_skew_sec: 0.06
    wbc_support_inset_m: 0.02
```

上述 WBC 参数是只读启动参数，修改后需有计划地重启运控，不能在线 `ros2 param set`。
状态消息为 `sensor_msgs/msg/JointState`，名称必须匹配模型，位置单位 rad/m，速度单位 rad/s 或 m/s，时间戳使用真实采集时间。
当前运控的 `/Control/MotorInfo` 不能直接改消息类型冒充此反馈，读取失败时填零的数据也不能作为实测值。

### 使用完整四轮支撑区域

本地新增 [运控参数片段](../src/motion_control_pkg/config/wbc_full_support_motion.yaml)，将以下两个参数一起设置：

```yaml
wbc_support_inset_m: 0.0
wbc_steering_uncertainty_margin_enabled: false
```

这会同时取消配置的内缩和转角不确定性补偿，使允许边界与模型四轮名义支撑多边形重合；越过该多边形仍会被 COM 检查拒绝。只把 `wbc_support_inset_m` 改成零，会继续保留默认转角补偿。

未知底盘转角仍明确标记为 assumed，不发布伪造转角反馈；动态 ZMP 仍须保持关闭。全支撑区域不再预留上述内缩量，且未包含额外载荷、地面接触变化等未建模因素。

查看该配置时使用 `wbc_full_support_observer.yaml`，其 `support_inset_effective_m` 应为 `0.0`；旧 `wbc_stage2_observer.yaml` 仍显示默认内缩区域。运控参数片段只包含两个边界参数，应合并到已有完整配置，不能覆盖驱动/工具参数。两个参数均在启动时读取。

此变更涉及 `libwbc_model.so` 及其调用方，部署时须完整重编和安装 `motion_control_pkg`，不能只换单个节点程序。参数仅在加载含此变更的新版本并重启后生效。

### 7.3 动态 ZMP

在静态 COM 已开启的基础上设置 `wbc_dynamic_zmp_enabled: true`。
动态模式要求完整、新鲜的实测 q/v，包含 `base_front_wheel_1_yaw_joint` 和 `base_rear_wheel_1_yaw_joint`；不能使用静态阶段的假设转向状态。
腿/躯干指定关节可按配置由连续位置估计速度；需等估计器有效，不能只发送一帧位置。
缺反馈、过期或时序不一致会拒绝动作，不应为让动作通过而伪造零反馈。
这是位置轨迹的动态参考与跟踪约束，不是力传感器实测 ZMP 闭环。

## 8. 普通关节运动与结果检查

### 8.1 普通单臂 MoveJ

以下角度同样仅为字段示例，执行前替换七个实际目标关节角（单位 rad）：

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
"{task_id: left_joint_example, arm_id: 0, control_mode: 0,
  joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L],
  position: [0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  enable_collision_check: 1, speed: 0.10, timeout_ms: 60000}" --feedback
```

右臂用 `arm_id: 1` 和 `Joint1_R` 到 `Joint7_R`。七个关节必须完整、唯一。
双臂 `MoveJoints` 用 `arm_id: 2`，填写 `left_joint_name/left_position/right_joint_name/right_position`，同步模式为 0。
头部/腿部关节路由为 `3/4`，但不能把 `MoveJoints.arm_id=5/6/7` 当作全身关节控制。

腿部 `arm_id: 4` 仅填写 `leg_joint1/leg_joint2/leg_joint3` 中两个关节时，
自动补偿第三关节，全程保持启动时的 body 朝向，允许位置变化；
原有四关节目标仍走普通 MoveJ。三种组合、任务 YAML 和保护条件见
[腿部定姿态 MoveJ](LEG_BODY_ORIENTATION_MOVEJ.md)。

### 8.2 判断动作结果

`Goal accepted` 只表示接单，之后还可能在 IK、点云导入、轨迹检查或执行阶段失败。
查看最终 Action 状态及 `success`、`error_message`，以及 `execution_time`、`final_distance_error`。
双臂/全身还可查看左右 TCP 误差。阶段耗时只有在 `timing_breakdown_valid=true` 时才有意义，不要把默认零值当作实测。

### 8.3 软件取消

下面会请求取消该 Action 服务的**所有当前目标**，不是硬件急停：

```bash
ros2 service call /Control/move_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
                 stamp: {sec: 0, nanosec: 0}}}"
```

正式客户端应持有 goal handle，针对自己的目标调用取消接口。
关闭 `send_goal` 终端不等于机器人已经停止；现场急停仍使用硬件通道。

## 9. 常见问题

| 现象/错误 | 检查重点 |
|---|---|
| `Goal rejected` | 目标 frame/四元数、speed、timeout、全身非零 acceleration/jerk、双臂同步模式、资源忙 |
| `Timeout waiting for synced robot vision` | 无缓存时最多等点云 5 秒；检查消息类型、持续发布、DDS/QoS 与话题参数 |
| `Invalid point cloud frame, dimensions or byte layout` | 空 frame、宽高、大小端、point_step/row_step/data 长度 |
| `Point cloud requires valid float32 x/y/z fields` | 小写 XYZ、FLOAT32、count=1、offset |
| `Unknown point cloud frame` | frame 是否在实际加载的模型中；上游是否正确变换到 base_link |
| `Point cloud contains no finite XYZ points` | 全 NaN/Inf、深度无效或筛选后为空 |
| 明明给了点云但没有环境避障 | 必须是模式 2；模式 0/1 不读外部点云 |
| 启用 COM 后拒绝运动 | 状态缺失/过期/时间差、当前 COM 越界、头/双臂忙、关节冻结条件 |
| `dynamic WBC requires complete fresh measured q/v including steering` | 动态模式缺转向实测或速度无效/含假设状态 |
| `bound_size` 改大后目标位置改变 | 全身代码将其作为 XYZ 偏移，不能拿来扩大避障包络 |

## 10. 对应代码

- [MovePose.action](../src/com_interfaces/action/MovePose.action)：字段和枚举。
- [main_node.cpp](../src/motion_control_pkg/src/main_node.cpp)：Action 路由、参数接收检查。
- [whole_body_module.cpp](../src/motion_control_pkg/src/whole_body_module.cpp)：全身 IK/轨迹、bound 偏移、COM/ZMP 开关。
- [scoped_collision_scene.hpp](../src/motion_control_pkg/include/motion_control_pkg/motion/scoped_collision_scene.hpp)：点云来源优先级与模式。
- [dual_arm_module.cpp](../src/motion_control_pkg/src/dual_arm_module.cpp)：持久订阅、后台缓存与最多 5 秒等待。
- [miracle_m_algo.cpp](../src/motion_control_pkg/src/miracle_m_algo.cpp)：`pointCloudToModelRoot()` 格式检查、1 cm Octree。
- [bringup.launch.py](../src/motion_control_pkg/launch/bringup.launch.py)：实际配置路径和启动参数。
- [model_node.cpp](../src/motion_control_pkg/src/wbc/model_node.cpp)：只读状态观察。
