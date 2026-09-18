# ylfrobot 任务节点命令参考

核对版本：`/home/elijah/ylfrobot`，`HHworkspace`，`96de196` 加本地插槽任务更新。
本文依据当前解析器、Action 客户端和服务端源码；示例未在机器人上执行。
实际动作中的位置、夹爪开度和速度需使用本机已经确认的参数。

当前最需要区分的三个事实：

- YAML 动作名为 **`move_pose`**。内部客户端虽然识别 `move_arm`，YAML
  解析器却没有对应参数分支，不能把它当作可用的 YAML 别名。
- 视觉结果自动缓存在内存中，后续动作固定取 `detected_targets[0]`。
  当前没有 `sort_by`、`target_index`、`target_no`、自动逐个取放等任务参数。
- 按确认的接口约定，视觉位姿已在 `base_link`，服务端输出标签也已修正。
  `expected_frame_id` 不执行坐标变换。旧视觉二进制需要更新才能使用新标签。

**任务动作与实际接口对应关系**

| YAML action_type | 实际 ROS Action 地址 | Action 类型 |
| --- | --- | --- |
| `vision_detect` | `/vision/detection_pose_cloud` | `com_interfaces/action/DetectionPoseCloud` |
| `move_pose` | `/Control/move_pose` | `com_interfaces/action/MovePose` |
| `move_joints` | `/Control/move_joints` | `com_interfaces/action/MoveJoints` |
| `gripper_ctrl` | `/Control/control_gripper` | `com_interfaces/action/ControlGripper` |
| `control_suck` | `/Control/control_suck` | `com_interfaces/action/ControlSuck` |
| `vision_check_self` | `/vision/Check_self` | `com_interfaces/action/HardwareCheck` |
| `vision_reset` | `/vision/Reset` | `com_interfaces/action/HardwareReset` |
| `control_check_self` | `/Control/Checkself` | `com_interfaces/action/HardwareCheck` |
| `control_reset` | `/Control/Reset` | `com_interfaces/action/HardwareReset` |
| `calibration_control` | `/vision/calibration_control` | `com_interfaces/action/CalibrationControl` |
| `auto_calibrate` | 任务层组合关节运动与标定请求 | 没有独立同名 Action |
| `move_compound` | 任务层并发调用子动作 | 子动作仅支持 `move_pose` / `move_joints` |

在终端直接调用 Action，与由任务节点按 YAML 调用，是两种入口。
直接调用会绕过任务层的资源锁、前置条件和序列结果缓存。

**加载环境、启动任务节点**

在实际运行机器上先加载 ROS 2 Humble 和当前版本的安装空间。
常规 colcon 构建后可使用 `source /path/to/ylfrobot/install/setup.bash`。
本次查看时，本地仓库还没有该 `install/`，上次合并的验证安装位于下面路径：
该旧安装不含本次插槽偏置扩展；插槽任务需按 [专用说明](SLOT_BOX_TASK.md) 构建更新。

```bash
source /opt/ros/humble/setup.bash
source /home/elijah/ylf_merge_20260917/install/com_interfaces/share/com_interfaces/local_setup.bash
source /home/elijah/ylf_merge_20260917/install/task_manager/share/task_manager/local_setup.bash

ros2 pkg prefix task_manager
ros2 action list -t
ros2 interface show com_interfaces/action/DetectionPoseCloud
ros2 interface show com_interfaces/action/MovePose
```

任务节点启动命令（已有同名节点时，先结束旧实例再替换）：

```bash
ros2 launch task_manager task_manager.launch.py \
  config_path:=/home/elijah/ylfrobot/src/task_manager/config/task_config.yaml
```

确认它实际加载的配置：

```bash
ros2 param get /task_manager_node config_path
```

`config_path` 只在启动时加载一次。修改主 YAML 或其外链任务文件后需重启任务节点。
使用显式源码路径时，单改 YAML 不要求重新编译；使用默认安装路径时，要更新
安装目录中的配置。仅执行 `ros2 param set` 不会触发重新解析。

默认服务监听 `0.0.0.0:12000`，Modbus 从站号为 `1`。
相机、视觉、运控 Action 服务需要已在相同 ROS 域中正常运行；本文不启动这些驱动。

**vision_detect：检测和查看返回值**

任务 YAML 片段：

```yaml
- action_type: vision_detect
  params:
    target_obj_name: "lefta"
    expected_frame_id: "base_link"
    priority: 10
    type: 0
```

终端直接调用一次：

```bash
ros2 action send_goal /vision/detection_pose_cloud \
  com_interfaces/action/DetectionPoseCloud \
  "{task_id: 'vision_manual_001', target_obj_name: 'lefta', expected_frame_id: 'base_link', priority: 10, type: 0}" \
  --feedback
```

返回 `success`、`status_code`、`error_message`、`total_detected_count` 和
`detected_targets[]`。每个目标含 `obj_name`、`pose`、`confidence`、`size`、
`bbox_center`、`tool_type`；位置单位 m，姿态顺序为四元数 `x,y,z,w`。
以每个 `target.pose.header.frame_id` 为准，不能只看 Result 顶层 header。

当前参数的实际效果：

| 参数 | 当前实现 |
| --- | --- |
| `target_obj_name` | 必须非空，用于日志和返回名称；包装层没有按此字符串筛选 NPU 类别 |
| `priority` | 允许 1–25；当前服务直接接受合法请求，未按它排序目标或抢占任务 |
| `type` | Action 注释约定 0 检测目标、1 检测放置；当前服务中的实际分支是 1 标记 `gripper`，其他值标记 `suction`，未据此切换取放算法 |
| `expected_frame_id` | 当前输出固定为 `base_link`；请求其他坐标系会告警，不会改写实际坐标系 |
| `require_single_target` | 新增 YAML 可选参数；为 true 时要求检测结果恰好包含一个目标 |
| `require_expected_frame` | 新增 YAML 可选参数；为 true 时核对目标实际 frame、有限位置及有效四元数 |

也可以查看逐个发布的目标消息：

```bash
ros2 topic echo /vision/target_detection
```

该话题的单条 `TargetDetection` 没有 task_id 和批次结束标记。需要准确保存
某一次检测的全部目标时，应保存 Action Result，而不是将连续话题消息当作一批。

**视觉位姿在任务节点内怎样保存、传递**

```mermaid
flowchart LR
    A[vision_detect 成功] --> B[动作 ActionContext 保存完整 Result]
    B --> C[行为树写入 seq1 或 seq2 缓存]
    C --> D[同序列 move_pose 读取 detected_targets 0]
    D --> E[use_config_pose false]
    E --> F[/Control/move_pose]
```

- `action_sequence1`、`action_sequence2` 是并行分支；每个分支内部的动作顺序执行。
  要让运动使用刚才的视觉结果，应把 `vision_detect` 和 `move_pose` 放在同一序列，
  检测在前、运动在后。不要把这两个有依赖关系的动作放进同一个 `parallel_actions`。
- 缓存保存整个检测数组，后续普通 `move_pose` 和 `move_compound` 的位姿子动作
  都取第 0 个目标。多次 `move_pose` 会重复用这个目标，不会自动递增、消费或删除。
- `use_config_pose: true` 使用 YAML 的固定目标；`false` 使用视觉缓存。
  `move_joints` 始终使用关节角配置，不会读取视觉目标。
- 缓存键为 `seq1`、`seq2` 等，不按 task_id 隔离。同序列新结果覆盖旧结果。
  当前 `clear_all_seq_vision_results()` 没有调用点，任务重置不清除此缓存；
  节点退出后才丢失。因此每次使用视觉驱动运动前，都应先在该序列完成检测，
  不能依赖“开始新任务会清空旧视觉位姿”。
- 外部 CLI 或本文保存脚本发起的检测，不会自动写入任务节点的这个缓存。

下面仅展示“同序列传递”的字段写法。它要求视觉已返回运控可识别坐标系下的
目标 TCP 位姿；如果视觉返回的是物体角点，应另外配置角点到 TCP 的偏置：

```yaml
action_sequencenum: 1
action_sequence1:
  - action_type: vision_detect
    params:
      target_obj_name: "lefta"
      expected_frame_id: "base_link"
      priority: 10
      type: 0
  - action_type: move_pose
    params:
      arm_id: 0
      use_config_pose: false
      frame_id: ""  # 保留视觉返回的真实 frame，不强行改标签
      target_pose: [0, 0, 0, 0, 0, 0, 1]  # 解析器仍要求字段；有视觉结果时不使用此值
      bound_size: [0, 0, 0]
      enable_collision_check: 1
      enable_move_line: false
      use_force_control: false
      hand_action_type: []
      speed: 0.1
      timeout_ms: 45000
```

视觉位姿已经在基坐标系，若检测给的是物体位姿，还需结合抓取偏置得到目标工具 TCP 位姿。
一般完整变换关系为：

`T_base_tcp_goal = T_base_vision_frame × T_vision_frame_object × T_object_tcp_goal`

这里的变换必须来自实际标定和抓取定义。本次新增单臂 `vision_pose_offset` 支持
“平移沿基坐标轴相加、姿态右乘局部四元数”，具体见 [插槽任务](SLOT_BOX_TASK.md)。
该模式检查视觉实际 frame 与配置一致，不会把相机坐标改标签当作基坐标。
双臂 `use_config_pose: false` 也不会分别选两个目标，而是把同一个第 0 目标
同时赋给左右目标，因此不应把它当作“双目标自动双臂抓取”。

**把视觉位姿保存到文件，再按规则排序**

附带示例脚本 [examples/capture_vision_poses.py](examples/capture_vision_poses.py)：
只向视觉发送一次检测请求，保存完整结果；不调用任何机械臂、夹爪或吸盘接口。

```bash
# 按置信度从高到低保存本次检测；目标名称按视觉配置填写。
python3 /home/elijah/ylfrobot/doc/examples/capture_vision_poses.py \
  --target lefta --frame base_link --type 0 \
  --sort confidence-desc --output /tmp/vision_poses_001.json

# 对已保存文件重新排序；此命令完全离线，不连接 ROS。
python3 /home/elijah/ylfrobot/doc/examples/capture_vision_poses.py \
  --input /tmp/vision_poses_001.json \
  --sort z-desc-x-asc --output /tmp/vision_poses_001_sorted.json
```

支持 `confidence-desc`、`x-asc`、`y-asc`、`z-asc`、`z-desc-x-asc`。
坐标排序要求所有目标具有同一个非空 frame；z 的大小按该坐标系定义，
不自动代表世界坐标中的高度。`z-desc-x-asc` 是按精确 z 降序、x 升序、y 升序，
未做高度分层容差聚类。

JSON 的 `targets` 保持视觉原始顺序，`sorted_targets` 保存排序副本，
`source_index` 是原始数组下标，不是稳定的物体 ID。文件存在时脚本拒绝覆盖。

保存和离线排序不改变任务节点缓存。若要在任务层自动按规则执行所有目标，
需要新增排序/选择/游标逻辑；现有 YAML 不支持配置这项行为。
当前可以查看选中目标，在完成坐标和 TCP 变换后，填入固定 `move_pose.target_pose`。

**move_pose：用户所说的 move_arm**

固定左臂目标的 YAML 写法（下面坐标仅示范格式）：

```yaml
- action_type: move_pose
  params:
    arm_id: 0
    frame_id: "base_link"
    use_config_pose: true
    target_pose: [0.35, 0.25, 0.80, 0, 0, 0, 1]
    bound_size: [0, 0, 0]
    enable_collision_check: 1
    enable_move_line: false
    use_force_control: false
    hand_action_type: []
    speed: 0.1
    acceleration_scale: 0.5
    jerk_scale: 0.5
    timeout_ms: 45000
```

对应终端调用：

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
  "{task_id: 'left_pose_001', arm_id: 0, control_mode: 0,
    target_pose: {header: {frame_id: 'base_link'}, pose: {
      position: {x: 0.35, y: 0.25, z: 0.80},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    bound_size: [0.0, 0.0, 0.0], enable_collision_check: 1,
    enable_move_line: false, use_force_control: false, hand_action_type: [],
    speed: 0.1, acceleration_scale: 0.5, jerk_scale: 0.5, timeout_ms: 45000}" \
  --feedback
```

`use_config_pose` 只属于任务 YAML，直接发 Action 时不填写它。

| 参数 | 含义/限制 |
| --- | --- |
| `arm_id` | 0 左臂，1 右臂，2 双臂，4 下肢；MovePose 的 5/6/7 为左/右/双臂联动下肢；HEAD=3 虽在接口定义中，当前 MovePose 路由拒绝它 |
| `target_pose` | `[x,y,z,qx,qy,qz,qw]`；位置 m，后四项为四元数，不是欧拉角 |
| `enable_move_line` | false 为 MoveP，true 为笛卡尔 MoveL |
| `enable_collision_check` | 0 关闭碰撞，1 自碰撞，2 自碰撞与外部点云；关节限位仍生效 |
| `speed` | `(0,1]` 的比例，不是直接指定 m/s |
| `acceleration_scale` / `jerk_scale` | 0 保持原比例，`(0,1]` 为额外缩放；普通头部/下肢目标不能设置非零值 |
| `bound_size` | 名称注释写“包络”，当前实现实际将它加到目标 xyz；示例用零，不应把它当作已建立负载碰撞模型 |
| `use_force_control` | 当前单/双臂普通位姿服务拒绝 true，示例保持 false |
| `hand_action_type` | 普通机械臂目标保持 `[]`；夹爪、吸盘用各自 Action |
| `timeout_ms` | 毫秒，包括规划/执行等待，按动作设置 |

模式 2 从显式 `env_point_cloud` 或 `/fusion/synced_robot_vision` 获得外部点云；
缺少有效点云会失败。这是规划时点云快照检查，不是运动中持续更新的动态避障。
当前 `DetectionPoseCloud` Result 不带环境点云，不能把目标位姿数组当作外部场景。

连续 MoveP 可给单臂 YAML 增加 `via_poses: [[x,y,z,qx,qy,qz,qw], ...]`；
最多 64 个，要求 `use_config_pose: true`、`enable_move_line: false`。
双臂用 `left_target_pose/right_target_pose`、`left_bound_size/right_bound_size`，
以及 `synchronization_mode: 0`；当前双臂服务不接受仅同步启动的模式 1。

**move_joints：按关节角运动**

```yaml
- action_type: move_joints
  params:
    arm_id: 0
    joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L]
    position: [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    enable_collision_check: 1
    speed: 0.1
    timeout_ms: 45000
```

这些角度仅示范格式。单臂需要完整且不重复的 7 个关节名称；单位 rad。
右臂用 `Joint1_R` 到 `Joint7_R`。头部 `arm_id: 3` 使用
`head_yaw_joint, head_pitch_joint`；下肢 `arm_id: 4` 使用
`leg_joint1, leg_joint2, leg_joint3, body_joint`。角度也需来自确认过的目标。

```bash
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
  "{task_id: 'left_joints_001', arm_id: 0,
    joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L],
    position: [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    enable_collision_check: 1, speed: 0.1, timeout_ms: 45000}" --feedback
```

**夹爪：gripper_ctrl / ControlGripper**

```yaml
- action_type: gripper_ctrl
  params:
    gripper_id: 0
    comport: [2, 2]
    position: [120, 0]
    speed: [30, 0]
    max_torque: [40, 0]
    block: true
    timeout_ms: 10000
```

终端调用同一设置：

```bash
ros2 action send_goal /Control/control_gripper com_interfaces/action/ControlGripper \
  "{task_id: 'left_gripper_001', gripper_id: 0,
    com_port: [2, 2], position: [120, 0], speed: [30, 0], max_torque: [40, 0],
    block_until_complete: true, timeout_ms: 10000}" --feedback
```

| 字段 | 说明 |
| --- | --- |
| `gripper_id` | 0 左、1 右、2 双夹爪 |
| 所有数组 | 顺序固定 `[左,右]`，即使单夹爪也必须至少两个元素 |
| `position` | 0–255；驱动接口约定 0 全开，255 全闭，120 是中间开度；不是 mm |
| `speed` / `max_torque` | 0–255 的设备参数，不是运动节点的 0–1 速度比例 |
| YAML `comport` / Action `com_port` | 2=COM1、3=COM2；1 是 CAN/CAN-FD，不能用于本夹爪串口控制 |
| YAML `block` / Action `block_until_complete` | 当前服务要求 true；false 会被拒绝 |

右夹爪示例改为 `gripper_id: 1` 并把目标放在数组第二项。
双夹爪设 `gripper_id: 2`，两项都填写实际目标；未选中的一侧不执行。

**吸盘：control_suck / ControlSuck**

```yaml
- action_type: control_suck
  params:
    suck_id: 0
    active: [true, false]
    timeout_ms: 5000
```

```bash
# 左吸盘开启。
ros2 action send_goal /Control/control_suck com_interfaces/action/ControlSuck \
  "{task_id: 'left_suck_on_001', suck_id: 0, active: [true, false], timeout_ms: 5000}" --feedback

# 左吸盘关闭；只有在允许释放当前负载时执行。
ros2 action send_goal /Control/control_suck com_interfaces/action/ControlSuck \
  "{task_id: 'left_suck_off_001', suck_id: 0, active: [false, false], timeout_ms: 5000}" --feedback
```

`suck_id` 0 左、1 右、2 双；`active` 固定 `[左,右]` 且至少两项。
`true` 开启、`false` 关闭，单侧命令不修改未选中的另一侧。
下层负责 GPIO 的低电平有效转换，调用者不用取反。
成功表示控制输出写入成功，当前 Action 没有真空压力反馈来证明已经吸牢。
任务层收到吸盘结果后还有约 1 秒固定等待，直接调用 Action 不包含这段任务层等待。

**顺序、并行与完整示例配置**

同一 `action_sequenceN` 内按列表顺序运行；不同 N 是并行分支。
某一阶段需要夹爪与吸盘并行时可以写：

```yaml
- parallel_actions:
    - action_type: gripper_ctrl
      params:
        gripper_id: 0
        comport: [2, 2]
        position: [120, 0]
        speed: [30, 0]
        max_torque: [40, 0]
        block: true
        timeout_ms: 10000
    - action_type: control_suck
      params:
        suck_id: 1
        active: [false, true]
        timeout_ms: 5000
```

全部子动作结束后才进入下一步。占用同一资源的动作仍会等待资源释放。
`delay_ms` 放在动作节点上，与 `action_type` 同级，表示该动作发送前延时。

附带完整配置 [examples/task_commands.yaml](examples/task_commands.yaml)，
包括纯视觉、固定姿态和工具并行三个独立任务。它没有被加入当前正式配置。
在确认样例参数并替换已有任务节点后，可显式加载：

```bash
ros2 launch task_manager task_manager.launch.py \
  config_path:=/home/elijah/ylfrobot/doc/examples/task_commands.yaml
```

这份示例的任务寄存器为：

| 示例任务 | 触发 | 状态 | mbpoll 的 -r（触发/状态） |
| --- | --- | --- | --- |
| vision_only | 40006 | 50006 | 6 / 10006 |
| fixed_pose_example | 40007 | 50007 | 7 / 10007 |
| tools_example | 40008 | 50008 | 8 / 10008 |

`mbpoll` 以下命令未使用 `-0`，`-r = 文档寄存器号 - 40000`。
服务端实际 Modbus PDU 地址则为 `文档寄存器号 - 40001`。
当前服务实现 FC06 单寄存器写入和 FC03 保持寄存器读取，不使用批量 FC16 触发。

```bash
# 示例配置已经加载，任务节点在本机；远程调用时替换 127.0.0.1。
# 0→1 上升沿触发一次 vision_only；它不调用运控。
mbpoll -m tcp -p 12000 -a 1 -t 4 -r 6 -1 127.0.0.1 0
mbpoll -m tcp -p 12000 -a 1 -t 4 -r 6 -1 127.0.0.1 1

# 读取本次任务状态。
mbpoll -m tcp -p 12000 -a 1 -t 4 -r 10006 -c 1 -1 127.0.0.1
```

状态：1 运行中、2 成功、3 失败；0 是尚无本次结果。触发寄存器完成后置 0。
运行状态在 100 ms tick 更新，刚写完触发就读取可能暂时看到上次的终态值。
执行期间不要重复触发；当前运行时收到的新上升沿可能进入 FIFO 等待队列。

当前正式配置的寄存器如下：

| 任务 | 触发寄存器 | 状态寄存器 |
| --- | --- | --- |
| move_box | 40001 | 50001 |
| control_suck_example | 40002 | 50002 |
| go_idle_pose | 40003 | 50003 |
| auto_calibrate | 40004 | 50004 |
| go_pick_pose | 40005 | 50005 |
| slot_box_right_1_left_3 | 40009 | 50009 |
| depalletize_pick | 40101 | 50101 |
| rack_place | 40102 | 50102 |
| rack_pick | 40103 | 50103 |
| pallet_place | 40104 | 50104 |
| demo1 | 40105 | 50105 |

新任务可在正式主配置 `tasks` 中注册 `config_file: tasks/your_task.yaml`，
路径相对主 YAML 所在目录。当前执行状态写入使用 `触发寄存器 + 10000`，
新增任务的 `status_reg` 也应保持这个关系。

**源码依据**

- 动作参数解析：[task_config_parser.cpp](../src/task_manager/src/task_config_parser.cpp)
- 动作路由、视觉缓存与第一个目标选择：[task_action_client.cpp](../src/task_manager/src/task_action_client.cpp)
- 序列构造和视觉结果入缓存：[behavior_tree_engine.cpp](../src/task_manager/src/behavior_tree_engine.cpp)
- 位姿选择与 frame 标签赋值：[task_action_client_arm.cpp](../src/task_manager/src/task_action_client_arm.cpp)
- 视觉参数实际效果、gripper_link 输出：[detection_pose_cloud_server.cpp](../src/vision_detection_pose_cloud/src/detection_pose_cloud_server.cpp)
- 夹爪校验：[gripper_command_validation.hpp](../src/motion_control_pkg/include/motion_control_pkg/execution/gripper_command_validation.hpp)
- 吸盘校验：[suck_command_validation.hpp](../src/motion_control_pkg/include/motion_control_pkg/execution/suck_command_validation.hpp)
- Modbus 触发和状态：[task_manager_node.cpp](../src/task_manager/src/task_manager_node.cpp)、[modbus_slave.cpp](../src/task_manager/src/modbus_slave.cpp)

离线检查：7 段 YAML 可解析，6 个直接 Action 请求可用本次生成的消息类型构造，
完整示例含 3 个任务且状态寄存器偏移正确；保存/排序脚本以合成 Result 数据验证了
置信度排序、坐标排序、保留原始顺序和 JSON 文件输出。未启动节点，未联机检测，
未执行机器人动作。这些检查不代替实际坐标标定、目标可达性或设备执行验收。
