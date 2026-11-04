# ylf_wbc 运控指令与参数完整说明



ros2 service call /Control/reset_motion_fault std_srvs/srv/Trigger '{}'

更新日期：2026-11-04。范围：当前本地 `ylf_wbc/src/motion_control_pkg` 的实际 ROS 入口、`com_interfaces` 的运控 Action、对应的任务 YAML 适配和只读 WBC observer。运行机器必须使用同版本接口与二进制；本文区分本地源码默认值与 本次核对的 x86 `it@192.168.10.18:/home/it/ylf_wbc` 源码与配置；每次上传、编译、安装、重启和实机验收仍须分别核对，不能由文档日期推定运行版本。

本次更新重点：

- [手臂与身体跟踪误差保护开关](#65-手臂与身体跟踪误差保护开关)：`arm_error`、`body_error` 独立配置，启动均默认关闭；单纯跟踪超限后等待静止及旧线程退出，再解除拦截。
- [六轴开机清错与 x86 安装](#24-六轴开机清错与-x86-安装)：启动清错由 27–30 扩展为 27–32，补入两个头部从站；仅在 OP/PDO 稳定后复位，再保持去使能等待正常使能。
- [身体运动的反馈快照](#111-身体运动反馈快照与采样前进)：身体倾角、升降读取仍新鲜的同一设备帧时，不再消耗 WBC 发布者的采样前进记录；设备采集时间与停止/到位的新帧要求保持有效。
- [4 ms 控制与进度发布隔离](#113-4-ms-控制与进度发布隔离)：发送循环只保存最新进度，独立 100 ms 定时线程发布；补充周期超时诊断及其他发布路径核查。


- [WBC 双臂 TCP 全程保持水平](#45-wbc-双臂-tcp-全程保持水平)：MovePose `arm_id=7`、`keep_tcp_level=true`，腿腰运动时双臂同步补偿；默认关闭，倾角容差默认 0.02 rad。包含直接 Action 和任务 YAML 示例。
- [腰腿和头部 0.002 rad 精定位](#35-腰腿和头部精定位)：普通 MoveJ 默认低速接近原始目标，以新鲜实测反馈确认到位；WBC 腰腿使用同一精定位判据，保留全身同步轨迹。双臂关节到位容差仍为 0.01 rad。
- 头部启动回零：默认低速回到 `[0,0]`，启动恢复专用 pitch 下界为 -0.7 rad；正常运动仍使用 URDF 限位，见第 2.2、8.6 节。

## 目录

本次同步范围为机器人上的 `com_interfaces`、`motion_control_pkg`、`onecube_robot_description` 及配套 x86 DriverSDK；不包含本地尚未部署的独立 LEG TCP 水平补偿扩展。

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
| 软件停止 | `/Control/stop_motion` | `std_srvs/srv/Trigger` | 锁存软件故障，拦截后续运动指令；停止确认异步进行 |
| 手臂跟踪误差开关 | `/Control/arm_error` | `rcl_interfaces/srv/SetParametersAtomically` | 双臂共用，默认关闭；由 `arm_error` 原子配置 rad 限制，见第 6.5 节 |
| 身体跟踪误差开关 | `/Control/body_error` | `rcl_interfaces/srv/SetParametersAtomically` | 腿、腰、头共用，默认关闭；由 `body_error` 原子配置 rad 限制，见第 6.5 节 |
| 清除软件运动故障 | `/Control/reset_motion_fault` | `std_srvs/srv/Trigger` | 资源释放、反馈健康且静止确认后清除软件锁存，不清驱动故障、不使能或自动恢复运动 |
| 在线驱动急停 / 清错 | `/Control/drive_control` | `com_interfaces/srv/DriveControl` | 天机左/右/双臂及 EtherCAT 腿/腰四轴；独立报告调用与反馈确认，保持停止、不自动使能 |
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

双臂安装外参、`body_link` 到 J1 的关系及 DH 约定见[全身 DH 报告第 3.1 节](WHOLE_BODY_DH_PARAMETERS.md#31-肩部安装变换)。沿用当前七轴模型时，安装标定使用 `body_link` 到 `arm-l-base_link/arm-r-base_link` 的固定变换；安装基座到 J1 的 0.1745 m 已包含在臂内 DH 中。

### 1.1 功能与参数速查

以下“默认”指接口或源码默认值；运行配置可以覆盖节点参数。速度、目标和超时应显式提供，消息初始化值不一定是有效请求。

| 功能 | 配置入口 | 当前支持范围及限制 |
|---|---|---|
| WBC 全程末端水平 | `keep_tcp_level=false` | 默认关闭；仅 MovePose 5/6/7 可设 true。TCP XY 相对固定且水平的 base_link 保持水平，允许竖直轴转向，禁止翻面 |
| 水平容差 | `tcp_level_tolerance_rad=0.02` | 单位 rad，有效范围 `[0.0001,0.1]`；约束最终 4 ms 指令点、点间插值及实测姿态 |
| WBC 腿腰补偿 | MovePose `arm_id=5/6/7` + 水平约束 | 腿腰与手臂使用联合时间参数化；5/6 约束对应单臂，7 约束双臂 |
| 仅腰参与 WBC | `whole_body_waist_only=false` | 设 true 时冻结腿 J1/J2/J3，只允许 WBC MoveP |
| 笛卡尔直线 | `enable_move_line=false` | 设 true 为 MoveL；支持单臂、双臂、WBC，也覆盖原地改变姿态 |
| 连续经过点 | `via_poses=[]` | 仅单臂配置位姿 MoveP，最多 64 点，中间点不要求停稳 |
| 双臂同步 | `synchronization_mode=0` | 同时开始和结束；值 1 虽有枚举但被拒绝 |
| 碰撞检查 | `enable_collision_check=0` | 1 自碰撞；2 自碰撞及环境点云；碰撞检查不等于每条路由都能自动绕障 |
| 目标位置偏移 | `bound_size` 及左右版本 | 在规划坐标系直接叠加 XYZ，实际未用作碰撞包络 |
| 速度及平滑程度 | `speed`、`acceleration_scale`、`jerk_scale` | speed `(0,1]`；后两者 `[0,1]`，0 沿用倍率 1，非零仅手臂 0/1/2 |
| 位置/阻抗模式 | `control_mode=0` | 1 对应手臂 MoveJ 关节阻抗、MoveP/MoveL 笛卡尔阻抗；须关闭水平约束，普通任务 YAML 不转发此字段 |
| 腿部定身体朝向 | LEG 两关节目标 | 第三腿关节补偿，保持身体起始朝向与腰角；身体位置可变 |
| 身体倾角/升降 | `leg_motion_mode=1/2/3` | 分别为绝对倾角、相对高度、绝对高度；独立腿部动作不支持 keep_tcp_level |
| 腰部相对旋转 | YAML `relative_position=false` | 设 true 仅用于 LEG 的单个 body_joint 简写，由任务层扩为四关节目标 |
| 腰腿/头部精定位 | `leg_precision.*`、`head_precision.*` | 启动参数，默认最终误差 ≤0.002 rad、连续稳定 200 ms；普通 MoveJ 有低速接近段，见第 3.5、8.5 节 |
| 头部启动回零 | `head_startup_home.enabled=true` | 一次启动回零；pitch 恢复下界 -0.7 rad，普通限位不变；期间阻止其他运动/工具任务 |
| COM/ZMP 支撑约束 | `wbc_static_com_enabled`、`wbc_dynamic_zmp_enabled` | 节点参数，均默认 false；动态依赖静态以及完整有效状态，独立于几何碰撞开关 |
| 视觉位姿目标及偏移 | YAML `use_config_pose`、`vision_*` | 视觉缓存、基准系/视觉局部平移、旋转、目标向下、保持当前 TCP 高度 |
| 动作编排与恢复 | YAML `parallel_actions`、`move_compound`、结果分支等 | 任务层功能，详见第 10 节；仍受底层资源和运动约束限制 |

WBC 启用水平约束时，要求起点已水平、目标与水平兼容。单独 LEG 动作不支持该约束，也不会自动扩成全身动作；需要手臂补偿时应使用 WBC。`force_vertical_down` 仅生成目标姿态；全程约束需在 WBC 动作中显式设置 `keep_tcp_level: true`。

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

ARM64 驱动及 NPU 视觉依赖应在相应平台准备。x86 可显式设置 `ONECUBE_USE_REAL_DRIVER_SDK=ON` 并指定配套 DriverSDK；未匹配真实驱动时可能使用 pseudo EtherCAT 后端。双臂 SDK 仍有实际连接逻辑，不能据此随意启动运控。只读 observer 使用下述独立入口。

`MoveJoints.action` 含身体运动字段，`MoveJoints.action` / `MovePose.action` 均含 `keep_tcp_level` 和 `tcp_level_tolerance_rad`，须按 `com_interfaces → motion_control_pkg → task_manager` 重建并安装消费者。不同接口版本不可混用。0.002 rad 精定位使用节点启动参数，无需向 Action 添加精度字段。本地逐包 CMake 安装可使用仓库根 [setup_local.bash](../setup_local.bash)，标准 colcon 安装使用 `install/setup.bash`。检查安装前缀：

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

当前代码默认开启一次**头部启动回零**：等待全身反馈有效且静止，校验路径后将 yaw/pitch 低速移到 `[0,0]`，到位后才开放正常运动和工具任务。启动可接受 pitch 起点最低 -0.7 rad；正常 yaw 范围仍为 `[-2.92,2.92]`、pitch 为 `[-0.62,0.41]` rad。失败锁存故障，不自动重试。默认速度上限 0.1 rad/s、总超时 180 s；关闭方式及其他启动参数见第 8.6 节和[头部启动回零说明](HEAD_STARTUP_HOMING.md)。

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

### 2.4 六轴开机清错与 x86 安装

2026-11-02 起，开机复位对象为 EtherCAT **27、28、29、30、31、32** 六个从站。原逻辑仅复位腿/腰 27–30；两个头部从站可能保留 `0xFF32`，使全身反馈校验和头部回零预检失败。启动先等待六从站 OP、PDO 完整且连续稳定 2 s，再发送 `enabled=-1` 约 200 ms，释放为 `enabled=0` 并确认故障清除。清错阶段保持实测位置，不执行使能或回零；后续正常使能与头部回零是单独步骤。日志应包含 `Startup fault reset alias 31/32` 和 `EtherCAT startup fault reset confirmed`，仍需读回故障码确认。

`0xFF32` 的含义应结合主站状态判断：运控退出后主站空闲、从站 PREOP，与运控运行期间 OP 下的故障是不同状态。`enabled=-1` 写入成功不等于错误已清除。

本次跟踪开关使用已有标准服务类型，未增加 MoveJoints/MovePose 字段，task_manager 无需因本次开关重编译。已配置好 SDK 和依赖的 x86 工作区可串行构建并回归：

```bash
cd /home/it/ylf_wbc
source /opt/ros/humble/setup.bash
source install/local_setup.bash
cmake -S src/motion_control_pkg -B build/motion_control_pkg -DBUILD_TESTING=ON
CMAKE_BUILD_PARALLEL_LEVEL=1 cmake --build build/motion_control_pkg --parallel 1
ctest --test-dir build/motion_control_pkg --output-on-failure -j1
```

常规更新先对旧运控请求正常退出，待 `shutdown=COMPLETE` 后再安装、重新 source 并启动。也可在维护授权下先备份并安装，但运行中的旧进程仍使用旧代码，必须另行关闭和重启；构建成功或覆盖安装均不代表新版本已运行。

```bash
cmake --install build/motion_control_pkg
source install/local_setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch motion_control_pkg bringup.launch.py \
  driver_config:=/opt/onecube/driver_sdk/configuration.xml
```

新进程的手臂及身体跟踪误差开关均从 OFF 开始。另开已 source 同一工作区、DDS 配置一致的终端执行第 6.5 节的 `arm_error`、`body_error` 命令。

本次已同步的头部恢复模板为 pitch 下界 `-0.7 rad`、命令速度上限 `0.1 rad/s`。普通限位依据加载的 URDF，当前 pitch 上界保持 `0.41 rad`；URDF 腰部速度上限为 `1.31 rad/s`，头部 yaw/pitch 均为 `0.61 rad/s`。

### 2.5 规划线程数与四线程启动

**`planning_worker_count:=4` 仍然可用。** 本地源码和 x86 已安装的 `bringup.launch.py` 都保留该参数，启动时将其作为整数传给运控节点。默认值为 **2**，仅接受 **1、2、4**；这是节点启动时的只读参数，修改文件或执行 `ros2 param set` 不会调整已有线程池，也不属于每个 Action/任务动作的 `params`。

生产 bringup 在 YAML 配置之后传入该 launch 参数，因此仅修改 YAML 中的同名字段可能被 launch 的默认值 2 覆盖；应在启动命令中显式指定。

| 参数值 | 规划行为 |
|---|---|
| `1` | 串行规划，不创建并行规划线程池 |
| `2`（默认） | 创建两个持久规划线程；双臂笛卡尔路径的左右臂 IK 可同时计算，较长路径的批量 FK、碰撞检查可分块并行 |
| `4` | 创建四个持久规划线程；较长路径的批量 FK、碰撞检查最多使用四线程；双臂 IK 仍只有左右两个任务，使用其中两个线程 |

四线程启动示例：先按第 2.4 节正常退出已有运控并确认 `shutdown=COMPLETE`，再在下一次启动时指定：

```bash
cd /home/it/ylf_wbc
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch motion_control_pkg bringup.launch.py \
  driver_config:=/opt/onecube/driver_sdk/configuration.xml \
  planning_worker_count:=4
```

各线程使用独立的 Pinocchio `Data`、IK 工作区和碰撞 `GeometryData`，共享只读模型与几何资源。协调线程仍串行管理规划调用和场景，线程池在所有活动任务结束后才返回或传播异常；参数设为 4 不表示四个运动 Goal 可以同时占用同一资源。

批量 FK 和批量碰撞检查仅在点数达到 `32 × planning_worker_count` 时启用线程池：2 线程至少 64 点，4 线程至少 128 点；更短的批处理走串行路径。双臂 IK 的两个任务不使用这个批量点数门槛。仅已接入这些接口的规划步骤使用该线程池，其他步骤仍按各自实现执行。增加线程只影响这些 CPU 规划阶段，不改变 ROS executor 线程数、4 ms 指令周期、URDF 速度限制、SDK 下发顺序或运动资源规则；实际收益取决于轨迹长度、规划耗时和 CPU 负载，4 线程不保证比 2 线程更快。

核对入口及启动后的配置：

```bash
ros2 launch motion_control_pkg bringup.launch.py --show-args
ros2 param get /motion_control_ns/motion_control planning_worker_count
```

四线程成功配置时，启动日志包含 `Planning workers: 4 (private caches, shared immutable model)`；对应运控进程的 Linux 线程名应包含 `wbc_plan_0`、`wbc_plan_1`、`wbc_plan_2`、`wbc_plan_3`。若存在同名节点导致参数服务返回空值，结合该运控进程的启动参数文件、启动日志和线程名交叉核对。

历史核对：2026-11-03 在 x86 `/home/it/ylf_wbc` 核对时，运行实例的启动值仍为 **2**，可见 `wbc_plan_0/1`；已安装 launch 的 `--show-args` 确认接受 1/2/4。本次 `parallel_planning` 离线回归通过，覆盖 2/4 持久线程并发、双臂 IK 串并行一致性、批量 FK、1/2/4 碰撞检查及取消/异常恢复。测试不连接厂商 SDK、不下发运动；本次文档更新未切换运行实例的线程数。

实现依据：[启动参数](../src/motion_control_pkg/launch/bringup.launch.py)、[节点初始化](../src/motion_control_pkg/src/main_node.cpp)、[并行规划](../src/motion_control_pkg/src/parallel_planning.cpp)、[持久线程池](../src/motion_control_pkg/include/motion_control_pkg/motion/planning_worker_pool.hpp)、[离线回归](../src/motion_control_pkg/test/parallel_planning_test.cpp)。

## 3. MoveJoints：手臂、头部、腰腿

### 3.1 Goal 全字段

除显式声明默认值的字段外，未赋值的消息字段一般为零、false、空字符串/空数组；这不代表是有效运动请求。`keep_tcp_level` 默认 false，水平容差默认 0.02 rad。建议总是明确填写任务 ID、速度、超时、碰撞模式与目标。

| 字段 | 类型/单位 | 含义与限制 |
|---|---|---|
| `header` | `std_msgs/Header` | 时间戳/元信息；身体专用模式 frame 留空或为 `base_link` |
| `task_id` | string | 业务追踪 ID，客户端建议非空且唯一 |
| `arm_id` | uint8 | 0/1/2/3/4，见总表 |
| `control_mode` | uint8 | 0 位置；1 手臂关节阻抗。头/腿普通旧路由不会因此成为阻抗控制；身体专用和双关节补偿要求 0 |
| `keep_tcp_level` | bool，默认 false | 所有 MoveJoints 均须为 false；仅 WBC MovePose 支持开启 |
| `tcp_level_tolerance_rad` | float64，默认 0.02 rad | 有限且在 `[0.0001,0.1]`；即使关闭约束也要求有效值 |
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
  position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
  speed: 0.1, acceleration_scale: 0.5, jerk_scale: 0.5,
  enable_collision_check: 1, timeout_ms: 30000}' --feedback

# 双臂同步 MoveJ
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: dual_j, arm_id: 2, control_mode: 0,
  left_joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L],
  left_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  right_joint_name: [Joint1_R, Joint2_R, Joint3_R, Joint4_R, Joint5_R, Joint6_R, Joint7_R],
  right_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  synchronization_mode: 0, speed: 0.1, enable_collision_check: 1,
  timeout_ms: 30000}' --feedback

# 头部 MoveJ：yaw、pitch
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: head_j, arm_id: 3, joint_name: [head_yaw_joint, head_pitch_joint],
  position: [0.0, 0.0], speed: 0.02, enable_collision_check: 1, timeout_ms: 30000}' --feedback


[motion_control_node-1] [WARN] [1793069645.906726196] [motion_control_ns.motion_control]: Motion admission refused: motion fault latched: invalid EtherCAT motor feedback: 31

# 腿腰关节目标使用四关节绝对目标；keep_tcp_level 必须为 false
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: leg_j, arm_id: 4,
  joint_name: [leg_joint1, leg_joint2, leg_joint3, body_joint],
  position: [-1.0, 2.2, -1.225, 0.0], speed: 0.1,
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

这些模式使用空 `joint_name/position`、位置控制、零额外 acceleration/jerk scale。下表描述独立身体运动；`keep_tcp_level` 必须为 false。

| 模式 | 目标 | 保持的量 |
|---|---|---|
| 1 | `body_tilt_deg`：身体 +Z 朝 `base_link +X` 倾斜为正，0 为竖直 | 起点 J1/J2/腰/双臂/头部关节，解算 J3 |
| 2 | `body_height_m`：相对起点的 Z 增量，正升负降 | 身体 X/Y、朝向、腰、双臂和头部关节 |
| 3 | `body_height_m`：`body_link` 原点在 `base_link` 的绝对 Z | 同模式 2 |

绝对身体高度不是 TCP 高度，也不是相机到地面距离。倾角 0°不等于 J3=0 rad。倾角模式不要给高度；升降模式不要给倾角。启用动态 ZMP 时当前身体专用模式拒绝执行，静态 COM 和碰撞检查仍可使用。

这些独立腿部 MoveJoints 模式不支持 `keep_tcp_level: true`；需要手臂补偿时使用 WBC MovePose。详见 [WBC 末端水平约束](TCP_LEVEL_CONSTRAINT.md)。

```bash
# 身体竖直
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: upright, arm_id: 4, leg_motion_mode: 1, body_tilt_deg: 0.0,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 300000}' --feedback

# 相对上升 2 cm；下降填 -0.02
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: lift, arm_id: 4, leg_motion_mode: 2, body_height_m: 0.02,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 30000}' --feedback

# 绝对身体高度示例
ros2 action send_goal /Control/move_joints com_interfaces/action/MoveJoints \
'{task_id: body_z, arm_id: 4, leg_motion_mode: 3, body_height_m: 1.0,
  speed: 0.1, enable_collision_check: 1, timeout_ms: 30000}' --feedback
```

身体专用运动预约上半身资源，要求完整、新鲜、无故障的反馈。当前默认到位要求腿腰各关节误差 ≤0.002 rad、身体朝向误差 ≤0.01 rad；升降还要求身体位置误差 ≤0.001 m。关节速度须 ≤0.01 rad/s，至少 5 帧递增反馈且连续稳定 200 ms；腿腰判据来自 `leg_precision.*`。FK 到位误差不是外部计量精度。运动定义见 [身体运动说明](LEG_BODY_MOTION.md)，最新到位策略见下一节。

### 3.5 腰腿和头部精定位

**0.002 rad（约 0.115°）是每个关节相对原始目标的实测到位容差**。当前代码默认启用，无需更改第 3.2 节的 Action 格式；任务 YAML 的腰部 J4/body_joint 简写展开后也使用腿腰策略。它不能直接换算成 TCP 毫米定位精度，也不改变 IK 求解阈值。

| 运动路径 | 默认最终关节误差 | 低速接近行为 |
|---|---|---|
| 普通腰腿 MoveJ，含腰 J4/body_joint | 每轴 ≤0.002 rad | 先接近，再执行一次低速末段 |
| 头部 yaw/pitch MoveJ | 每轴 ≤0.002 rad | 同上 |
| 腿部 MovePose、两关节定身体朝向、身体倾角/升降 | 腿腰每轴 ≤0.002 rad | 沿原约束路径执行，收紧实测到位判据 |
| WBC MovePose（5/6/7）的腿腰 | 腿腰每轴 ≤0.002 rad | 保留手臂与腿腰同步轨迹，不独立重放腿腰末段 |
| 单臂、双臂及 WBC 中的手臂 | 每轴 ≤0.01 rad | 保持原有手臂完成策略 |

普通腰腿/头部 MoveJ 的默认执行过程：

1. 按起点与目标的**最大单关节位移**判断是否分段。大于 0.03 rad 时，先到达离目标最大关节差为 0.03 rad 的接近点；小于等于 0.03 rad 时整段低速。
2. 分段动作在接近点以规划零速度、零加速度结束第一段，并等待实测位置进入接近点 0.01 rad 范围、速度和稳定时间满足条件。这是进入末段的交接判据，最终仍必须满足 0.002 rad。
3. 执行已经规划和校验的低速末段，目标保持原始关节角。默认末段速度取原规划速度上限的 20%，并限制在 0.03 rad/s 以内；加速度、jerk 也按 20% 缩放，分别不超过 0.2 rad/s²、0.5 rad/s³，同时遵守 URDF 速度限制。
4. 末点下发后持续检查实测反馈：所有目标关节误差 ≤0.002 rad、实测及差分速度 ≤0.01 rad/s、至少 5 帧有效递增反馈，并连续稳定 200 ms，才返回成功。

两段采用 Ruckig 规划，保持原有碰撞及跟踪检查，并校验实际 4 ms 指令轨迹。低速末段是本次动作预先规划的一部分，只执行一次；不会在到位失败后自动追加目标偏置、循环补发或重新发起动作。等待期间保持最后目标。

如果已经静止但误差仍超限，连续 1.5 s 没有至少 0.0002 rad 的改善，则返回 `completion stalled at original target`。错误信息包含最大误差、最差关节索引、目标、实测和容差；索引从 0 开始，腿腰固定顺序中的 J4/body_joint 对应 3。持续静差应检查驱动位置环、负载、摩擦、零位和反馈分辨率，降低规划速度本身不能保证消除静差。

独立腰腿/头部每次到位等待默认最多 5 s；WBC 保持原来的 3 s 完成等待上限，两者都受 `timeout_ms` 总预算限制。分段接近和 200 ms 稳定确认会增加耗时；剩余预算不足时拒绝继续下发。取消、反馈失效、驱动异常或命令拒绝仍按原有失败处理退出。

可配置项见[精定位启动参数](#85-精定位启动参数)，完整实现、验证入口与边界见[腿腰和头部精确到位](PRECISION_ARRIVAL.md)。0.002 rad 是编码器反馈的成功判据，真机可达精度仍需实测确认。

## 4. MovePose：MoveP、MoveL、双臂和全身

### 4.1 Goal 全字段

| 字段 | 类型/单位 | 含义与限制 |
|---|---|---|
| `header` / `task_id` | Header / string | 顶层追踪信息，目标坐标系在各 PoseStamped 内 |
| `arm_id` | uint8 | 0/1/2 手臂，5/6/7 全身；旧 LEG=4 见下文；HEAD=3 不支持 |
| `control_mode` | uint8 | 0 位置；1 手臂笛卡尔阻抗；全身的腰腿仍下发位置轨迹 |
| `keep_tcp_level` | bool，默认 false | 仅 arm_id=5/6/7 且 control_mode=0 可设 true；所选手臂补偿腿腰运动，全程水平；起点和目标须已满足 |
| `tcp_level_tolerance_rad` | float64，默认 0.02 rad | 有限且在 `[0.0001,0.1]`；以 base_link 为参考，允许竖直轴转向但禁止翻面 |
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
    position: {x: 0.22295545050928492, y: 0.458363216197255, z: 1.0478599556914261},
    orientation: {x: 0.0031107532158121828, y: 0.9999639233833763, z: -0.006919606196158049, w: -0.003820235103448852}}},
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

### 4.5 WBC 双臂 TCP 全程保持水平

启用方式是 **MovePose `arm_id=7` + `keep_tcp_level=true` + 位置模式 `control_mode=0`**。腿腰姿态变化时，双臂同步调整关节，保持各自工具 TCP 的 XY 平面平行于 `base_link` 的 XY 平面。约束对象是 TCP 朝向；手臂各连杆不需要水平，也不要求两个 TCP 同高或保持固定间距。

| 配置 | 作用 |
|---|---|
| `keep_tcp_level: true` | 显式开启；省略或 false 则关闭 |
| `tcp_level_tolerance_rad: 0.02` | TCP 法向倾角容差，约 1.15°；有限值且在 `[0.0001,0.1]` rad 内，关闭时也须合法 |
| `arm_id: 7` | 双臂 TCP 同时受约束；5/6 只约束左/右 TCP |
| `enable_move_line: false` | WBC MoveP，补偿中间路径的 TCP 倾角 |
| `enable_move_line: true` | WBC MoveL，同时保持直线几何要求 |
| `whole_body_waist_only: true` | 仅 MoveP 可用，固定腿 J1/J2/J3，由腰 J4 和所选手臂联动 |

动作开始时，所选 TCP 必须已经水平；每侧按自己的起点选择 TCP Z 朝上或朝下分支，目标也须在该分支内。允许绕竖直轴转向，禁止翻面。若起点不水平，应先用关闭约束的准备动作到达经核实的水平姿态，再开启本功能；系统不会自动插入调平动作或改写不兼容目标。

以下直接 Action 示例假定左右 TCP 起点均为 Z 朝下，目标四元数 `[1,0,0,0]` 也为 Z 朝下。位置仅演示格式，使用前须替换为现场可达目标；若某侧起点朝上，该侧目标也应选朝上姿态，例如 `[0,0,0,1]`。

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{task_id: wbc_dual_tcp_level, arm_id: 7, control_mode: 0,
  left_target_pose: {header: {frame_id: base_link}, pose: {
    position: {x: 0.4, y: 0.25, z: 1.0},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}},
  right_target_pose: {header: {frame_id: base_link}, pose: {
    position: {x: 0.4, y: -0.25, z: 1.0},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}},
  left_bound_size: [0.0, 0.0, 0.0], right_bound_size: [0.0, 0.0, 0.0],
  synchronization_mode: 0, enable_move_line: false, whole_body_waist_only: false,
  keep_tcp_level: true, tcp_level_tolerance_rad: 0.02,
  use_force_control: false, speed: 0.1,
  enable_collision_check: 1, timeout_ms: 60000}' --feedback
```

改为 MoveL 时将 `enable_move_line` 设 true，并保持 `whole_body_waist_only=false`。任务 YAML 的完整动作片段见第 10.3 节。WBC 根据 TCP 目标解算腿腰与手臂，接口没有同时指定某个腰腿关节角的字段；独立 LEG MoveJ 不支持开启 `keep_tcp_level`；本次发布仅包含 WBC MovePose 的水平约束。

全程约束使用实际 `left_arm_tool_tf/right_arm_tool_tf`，覆盖最终 4 ms 指令点、点间关节插值及执行/到位期间的实测 TCP 姿态。腿腰与手臂共用时间参数，不能只检查起终点水平。普通手臂 `arm_id=0/1/2`、头部、独立腿部 MovePose 均不支持启用；MoveJoints 仅 `arm_id=4` 支持；阻抗模式也会被拒绝。水平补偿不可行、越限、碰撞或监控异常时动作失败，不转为关闭约束执行。

参考系为固定 `base_link`，底盘须固定且水平；当前没有 IMU 重力补偿。该约束也没有加入双臂刚性抓持的固定相对位姿或受力约束。`force_vertical_down` 只生成视觉目标姿态，不能代替全程水平约束。

**水平倾角容差 0.02 rad 与关节精定位容差 0.002 rad 分别配置。** 即使收紧 `leg_precision.goal_tolerance_rad`，也不会自动改变 `tcp_level_tolerance_rad` 或手臂关节容差。两项功能可在同一个 WBC 动作中共同生效：路径保持 TCP 水平，结束时腿腰按精定位判据到位。详细定义及源码入口见 [WBC 末端水平约束](TCP_LEVEL_CONSTRAINT.md)。

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

`Goal accepted` 仅表示接单，不能代表 IK 可达、轨迹可下发或实测到位。WBC 末点下发后仍等待参与手臂和四个腰腿关节的实测反馈，当前默认判据为：

| 部位 | 关节位置误差 | 实测及差分速度 | 有效反馈及稳定要求 |
|---|---|---|---|
| 所选手臂 | 每轴 ≤0.01 rad | ≤0.01 rad/s | `low_speed=true`，至少 5 帧新反馈、连续 100 ms |
| 腿腰 | 每轴 ≤0.002 rad | ≤0.01 rad/s | 至少 5 帧新反馈、连续 200 ms，读取 `leg_precision.*` |

WBC 同时检查由实测关节 FK 得到的所选 TCP 目标误差：位置 ≤0.003 m、姿态 ≤0.02 rad。开启 `keep_tcp_level` 后，到位等待期间水平监控也保持有效。这些 TCP 判据与腰腿 0.002 rad 关节判据必须分别满足。

WBC 完成等待最多 3 s，同时受 Action 总超时限制；腿腰精定位参数中的 `completion_timeout_sec` 不改变这个 3 s 上限。反馈序号 100 ms 不前进、驱动错误、非有限值、腿腰静差停滞等都会失败。手臂完成判据及 WBC 3 s 上限保持代码常量；腰腿/头部使用第 8.5 节的启动只读参数。

单臂路径 `timeout_ms<=0` 内部有 5000 ms 回退，双臂/WBC 要求正超时。当前头部和普通腿路径也已将动作开始时间计入轨迹下发及到位等待的截止时间，正超时不足时拒绝下发或结束执行。规划/SDK 调用不承诺硬实时可抢占，统一建议显式给正数并处理 Action 终态。

单臂、双臂、腰腿和工具分别有资源占用；双臂整体运动要求左右都空闲，全身及身体模式还会占用相应腰腿/上半身资源。任务层并行并不代表两个动作可以争用同一资源。

### 6.3 软件取消

正式客户端应保留 GoalHandle 并只取消自己的目标。下面请求取消 `/Control/move_pose` 的所有目标：

```bash
ros2 service call /Control/move_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
'{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
```

MoveJ 改服务路径为 `/Control/move_joints/_action/cancel_goal`。取消请求获准后仍需观察终态与实测停止。退出 `ros2 action send_goal` 终端不等于停止；软件取消不替代硬件急停。

### 6.4 全局软件停止和故障锁存

`/Control/stop_motion` 与 `/Control/reset_motion_fault` 均为 `std_srvs/srv/Trigger`，请求体为空。停止服务安装软件指令拦截并锁存故障；返回成功表示已提交该软件操作，实测静止确认异步进行，应观察 `/Control/motion_safety`。它不提供已验证的驱动 Quick Stop、STO 或制动控制。

复位服务要求运动资源全部释放、旧执行线程退出、反馈健康且已确认静止，仅清除软件锁存，不清除驱动故障、不重新使能、不自动继续旧轨迹。实测反馈过期、腿/头跟踪误差越界、水平约束越界、下发失败、执行中取消/超时等均可能触发软件故障。机械臂跟踪功能默认关闭；显式开启后的单纯机械臂跟踪超限按第 6.5 节处理，不形成持久锁存。

在线驱动维护使用 `/Control/drive_control`（`com_interfaces/srv/DriveControl`）：

| 字段 | 值与语义 |
|---|---|
| `target` | `0` 左臂 A；`1` 右臂 B；`2` 双臂；`3` 腿/腰四轴（J1/J2/J3/腰，EtherCAT 别名 30/29/28/27） |
| `operation` | `0` 在线急停；`1` 清除驱动故障 |
| `command_issued`（响应） | SDK 已调用或总线已写入；不能单独证明硬件停止/清错 |
| `feedback_confirmed`（响应） | 新鲜且前进的设备帧确认去使能和静止；清错另检查错误码 |
| `success`（响应） | 只有调用和反馈均确认才为 true |
| `message`（响应） | 原始驱动状态/错误码或拒绝、超时原因 |

服务先锁存整机运动，阻断 WBC 等旧轨迹继续下发，再针对选中驱动操作。手臂使用 `EStop("A"/"B"/"AB")` 和 `OnClearErr_A/B()`；清错检查控制器状态与各关节 `SERVO*ERR*` 返回值。腿/腰在现有总线线程中使用 `enabled=0` 去使能、`enabled=-1` 清错 200 ms 后保持 `enabled=0`；不改变头部目标。清错要求资源空闲、部件已静止且处于去使能或故障状态，急停无需先满足这些条件。

确认门限：反馈年龄 ≤100 ms，至少 5 组新帧连续稳定 ≥100 ms，速度 ≤0.01 rad/s，窗口位移 ≤0.001 rad。缺少真实 PDO 时间戳的旧腿部 SDK 不允许清错，也不能确认停止；不会将重复读缓存计作新帧。急停可中断同类驱动的清错等待。该网络操作不提供硬件急停/STO/抱闸确认，也不自动使能、软件复位或继续运动。调用后应分别检查请求返回值与实测反馈确认。

### 6.5 手臂与身体跟踪误差保护开关

`arm_error` 控制左右手臂；`body_error` 统一控制三个腿关节、腰部 `body_joint`、头部 yaw/pitch。两组相互独立，**每次进程启动均默认 OFF**，仅在运动资源空闲且没有待处理停止/故障时允许切换。

所有经过共用发送门的运动使用相同规则，包括 MoveJ、MoveP/MoveL、受支持的位置/阻抗模式、预规划轨迹及 WBC 输出。任务节点无需在 Goal 或 YAML 中新增字段。旧 `error` 命令及 `/Control/arm_tracking` 服务已分别更名为 `arm_error` 和 `/Control/arm_error`。

```bash
cd /home/it/ylf_wbc
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

arm_error status          # 查看两组实际开关、限制与整机安全状态
body_error status
arm_error 0.02 0.08        # 开启双臂跟踪误差保护，单位 rad
body_error 0.02 0.08       # 开启腿、腰、头的跟踪误差保护，单位 rad
arm_error off
body_error off
```

直接输入命令名或追加 `on` 会在交互终端提示填写限制；无交互输入时必须提供限制。只填第一个数时，立即停止限制默认为 `0.08 rad`。`ros2 run motion_control_pkg arm_error ...` 与 `ros2 run motion_control_pkg body_error ...` 为等价入口。

| 配置 | 输入与含义 |
|---|---|
| `enabled` | 各组独立开关，每次进程启动默认 OFF |
| `position_error_rad` | 第一个数；误差严格大于该值并持续 `motion_tracking_dwell_sec`（默认 80 ms）时终止本次任务 |
| `hard_error_rad` | 第二个数；误差严格大于该值时立即拦截后续轨迹点，不等待持续时间 |
| 数值校验 | 必须有限，且 `0 < position_error_rad <= hard_error_rad`；非法输入不会改变原配置 |
| 参考量 | 按实测采集时刻匹配此前实际发送的关节指令；控制器指令回显仅用于诊断 |
| 保持监控 | 开启某组后，其他链运动期间该组未参与运动的关节也以实测保持参考使用同一组限制 |

服务 `/Control/arm_error` 和 `/Control/body_error` 均使用 `rcl_interfaces/srv/SetParametersAtomically`，开启时原子提交 `enabled=true` 与两个 rad 限制，关闭时只提交 `enabled=false`。它们不是通用节点参数更新接口。`motion_tracking_error_rad` 与 `motion_tracking_hard_error_rad` 仅提供身体组初始限制值，不会开启 `body_error`；运行时开关及当前实际限制以 `/Control/motion_safety` 为准。

两组的单纯位置跟踪超限采用同一处理流程：

1. 拦截后续轨迹点，保持最后已发送指令，本次 Action 返回失败，相关旧执行线程结束。
2. 用新鲜、递增的设备反馈确认所有已配置链静止：至少 5 帧、连续 100 ms，实测及位置差分速度不超过 `0.01 rad/s`。
3. 静止确认且所有旧执行线程退出后，自动解除本次跟踪拦截，下一任务无需 `/Control/reset_motion_fault`。旧轨迹不自动继续。

停止期间状态为 `ERROR_TRACKING_STOPPING`、`ERROR_TRACKING_STANDSTILL_CONFIRMED` 或 `ERROR_TRACKING_STOP_UNCONFIRMED`。未确认停止时继续阻止新任务。恢复后回到 `MONITORING`，`last_error_stop` 保留带 `arm_error` 或 `body_error` 标记的原因；状态还分别报告两组的 ON/OFF、`*_limit_rad` 与 `*_hard_limit_rad`。

开关仅控制相对已发送指令的位置误差门槛及普通运动中的保持偏移检查。URDF 位置/速度限制、反馈有效性和年龄、驱动错误、最终到位要求（腿/腰/头仍为 `0.002 rad`）、TCP/碰撞约束、显式停止及启动回零的专用约束继续执行。这些实际故障仍锁存；若跟踪停止期间出现这类故障，停止升级为普通故障，不能自动解除。软件停止不代表驱动 Quick Stop、STO 或抱闸确认。

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

### 8.4 实测运动监控参数

这些是运控节点启动参数，不是每个 Action 的字段。机械臂跟踪开关和两个 rad 限制独立通过第 6.5 节配置：

| 参数 | 源码默认 | 含义 |
|---|---|---|
| `motion_tracking_error_rad` | 0.02 | 身体组初始持续误差限制，不自动开启；以 body_error 状态为准 |
| `motion_tracking_hard_error_rad` | 0.08 | 身体组初始立即停止限制，不自动开启；以 body_error 状态为准 |
| `motion_tracking_dwell_sec` | 0.08 | 普通误差超过阈值的持续时间 |
| `motion_feedback_max_age_sec` | 0.1 | 最大有效反馈年龄 |
| `motion_stop_confirmation_timeout_sec` | 3.0 | 软件故障后等待实测静止的时间预算 |

仅显式启用约束的 WBC 动作注册水平监控，使用该动作的 `tcp_level_tolerance_rad`；所需手臂及腿腰反馈的跨通道时间差上限为 20 ms。URDF 关节限位、控制器限位、轨迹导数检查、实测反馈与到位检查不依赖 `enable_collision_check`。

### 8.5 精定位启动参数

腰腿前缀为 `leg_precision`，头部前缀为 `head_precision`，各自独立。例如完整参数名为 `leg_precision.goal_tolerance_rad`。WBC 读取同一套腿腰完成判据，双臂不读取这两组参数。以下均为**节点启动时只读参数**，不能放入任务动作 `params` 或通过 `ros2 param set` 热更新。

| 参数后缀 | 默认值 | 含义及范围 |
|---|---|---|
| `goal_tolerance_rad` | 0.002 | 最终每轴位置误差，`(0,0.1]` rad |
| `stopped_velocity_rad_s` | 0.01 | 实测及差分速度上限，`(0,0.1]` rad/s |
| `stable_time_sec` | 0.2 | 连续稳定时间，`[0.1,5]` s；同时至少 5 帧新反馈 |
| `stall_time_sec` | 1.5 | 静止后无有效改善的判定时间，`[0,30]` s；0 关闭停滞诊断，超时仍有效 |
| `minimum_progress_rad` | 0.0002 | 重置停滞计时的最小改善量，必须 >0 且小于最终容差 |
| `completion_timeout_sec` | 5.0 | 独立腰腿/头部每次到位等待上限，须 ≥稳定时间且 ≤30 s；仍受 Action 总超时限制 |
| `approach_enabled` | true | 普通 MoveJ 低速接近开关；false 只关闭低速接近，保留最终精度要求 |
| `approach_distance_rad` | 0.03 | 最大关节差定义的接近距离，须 >最终容差且 ≤0.1 rad |
| `approach_speed_scale` | 0.2 | 低速段速度、加速度、jerk 的缩放，`(0,1]` |
| `approach_velocity_rad_s` | 0.03 | 低速段速度上限，`(0,0.1]` rad/s |
| `approach_acceleration_rad_s2` | 0.2 | 低速段加速度上限，`(0,1]` rad/s² |
| `approach_jerk_rad_s3` | 0.5 | 低速段 jerk 上限，`(0,5]` rad/s³ |

完整配置模板见 [precision_arrival.yaml](../src/motion_control_pkg/config/precision_arrival.yaml)，默认值已在代码中启用。需要覆盖时，将对应配置合入实际启动读取的 YAML 的 `ros__parameters` 下；生产 bringup 的配置路径见第 2.2 节，仅修改仓库模板不会更改运行节点。

```yaml
/motion_control_ns/motion_control:
  ros__parameters:
    leg_precision:
      goal_tolerance_rad: 0.002
      stable_time_sec: 0.2
      approach_enabled: true
    head_precision:
      goal_tolerance_rad: 0.002
      stable_time_sec: 0.2
      approach_enabled: true
```

只读核对当前节点的实际参数：

```bash
ros2 param get /motion_control_ns/motion_control leg_precision.goal_tolerance_rad
ros2 param get /motion_control_ns/motion_control leg_precision.approach_enabled
ros2 param get /motion_control_ns/motion_control head_precision.goal_tolerance_rad
```

如果参数不存在，先核对运行节点及安装前缀是否为新版本。到位精度与第 8.4 节的运行跟踪误差阈值分开配置；精定位不会把 `motion_tracking_error_rad` 自动改为 0.002。

### 8.6 头部启动回零参数

前缀为 `head_startup_home`，以下同样为启动时只读参数。边界数组顺序均为 `[yaw,pitch]`。

| 参数后缀 | 默认值 | 含义 |
|---|---|---|
| `enabled` | true | 启用一次启动回零；false 恢复普通 URDF 限位检查且不自动回零 |
| `recovery_lower_rad` | `[URDF yaw 下界,-0.7]` | 当前模型为 `[-2.92,-0.7]` rad，只允许扩展负方向 pitch |
| `recovery_upper_rad` | URDF 上界 | 当前模型为 `[2.92,0.41]` rad，不允许扩大 yaw 或正向 pitch 范围 |
| `velocity_rad_s` | 0.1 | 回零速度上限，还受 URDF 速度限制 |
| `acceleration_rad_s2` | 0.05 | 回零加速度上限 |
| `jerk_rad_s3` | 0.1 | 回零 jerk 上限 |
| `feedback_wait_sec` | 30.0 | 等待全身有效静止反馈的上限 |
| `timeout_sec` | 180.0 | 包含启动等待、规划、执行和到位的总超时 |

回零到位至少要求两轴误差 ≤0.002 rad、5 帧递增反馈、连续稳定 200 ms；若 `head_precision` 配置更严格则采用更严格条件。启动例外在回零成功或失败后收回，URDF 和正常 MoveJ 限位不变。恢复流程、失败处理与配置示例见 [HEAD_STARTUP_HOMING.md](HEAD_STARTUP_HOMING.md) 和 [head_startup_home.yaml](../src/motion_control_pkg/config/head_startup_home.yaml)。

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
| `keep_tcp_level` | 默认 false；仅 move_pose 的 arm_id=5/6/7（包括复合动作中的 WBC 子动作）可显式设 true；其他动作设 true 被拒绝 |
| `tcp_level_tolerance_rad` | 默认 0.02，有限且在 `[0.0001,0.1]` rad |
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
| `on_success` / `on_failure` | 动作级结果分支，可配置后续动作和成功/失败处理策略 |
| `feedback_check` / `retry_actions` | 动作级反馈条件检查及调整重试；主要读取视觉结果字段，不等于关节闭环跟踪监控 |
| `ik_failure_recovery` / `if_recovery_applied` | 受限的 WBC IK 失败升高重试及条件下降恢复，见下一节 |

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

**双臂 TCP 全程保持水平的 WBC MoveP**：下例与第 4.5 节 Action 对应，左右起点均须已满足 TCP Z 朝下的水平姿态。七元素目标数组依次为 `[x,y,z,qx,qy,qz,qw]`；示例坐标需替换为现场核实的可达值。

```yaml
- action_type: move_pose
  params:
    arm_id: 7
    use_config_pose: true
    frame_id: base_link
    left_target_pose: [0.4, 0.25, 1.0, 1.0, 0.0, 0.0, 0.0]
    right_target_pose: [0.4, -0.25, 1.0, 1.0, 0.0, 0.0, 0.0]
    left_bound_size: [0.0, 0.0, 0.0]
    right_bound_size: [0.0, 0.0, 0.0]
    synchronization_mode: 0
    use_force_control: false
    enable_move_line: false
    whole_body_waist_only: false
    keep_tcp_level: true
    tcp_level_tolerance_rad: 0.02
    enable_collision_check: 1
    speed: 0.1
    timeout_ms: 60000
```

使用 MoveL 时改 `enable_move_line: true`；仅腰参与的 MoveP 改 `whole_body_waist_only: true`。本动作的腰腿默认按 0.002 rad 判定到位，手臂仍为 0.01 rad；无需在任务动作里填写 `leg_precision` 或额外的精定位开关。

下面为原有的单臂加腰 MoveP 与独立身体升降片段，未启用 TCP 水平约束。独立升降动作不会自动补偿手臂 TCP 姿态。

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

按当前 `task_manager` 的 `MODBUS_PROTOCOL_ADDR_OFFSET=40001`，配置寄存器号减 40001 为 Modbus 原始零基地址：40010→9，50010→10009。mbpoll 明确加 `-0`，避免默认一基寻址混淆。

```bash
# 只读查询任务状态：0 空闲、1 执行中、2 成功、3 失败
mbpoll 127.0.0.1 -p 12000 -a 1 -t 4 -0 -r 10009 -c 1 -1

# 触发笔记本任务，会实际执行整个任务流程
mbpoll 127.0.0.1 -p 12000 -a 1 -t 4 -0 -r 9 -1 1
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
| `/Control/motion_safety` | 软件停止/故障状态、静止确认、反馈年龄、误差、指令计数及 arm_error/body_error 开关/限制，String 类型 |
| `/Control/shutdown_state` | x86 正常退出阶段：RUNNING、STOPPING、DISABLING、DISABLED_STATIONARY_CONFIRMED、COMPLETE 或 FAILED_HOLD |
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
| TCP level start / target tilts or flips | 起点不水平，或目标倾角/朝上朝下分支不兼容；核对实际 TCP 外参及 base_link 下的目标姿态 |
| TCP level projection / interpolation / tilt 超差 | 路径水平补偿、插值认证或实测监控未通过；核对当前关节、限位、轨迹及反馈时序 |
| 外部点云等待失败 | SyncedRobotVision 类型、配置话题、QoS、持续有效发布，当前最多等待 5 s |
| whole-body completion timed out | 已经进入末点到位阶段；看误差、速度、low_speed 和反馈序号，非 IK 阶段 |
| completion stalled at original target | 已静止但残余误差超出精定位容差，1.5 s 无至少 0.0002 rad 改善；检查 worst_joint_index、目标/实测、驱动位置环和负载 |
| precision approach handover / insufficient timeout for precision suffix | 接近点未稳定或剩余预算不足；检查反馈与 timeout_ms，末段不会自动重试 |
| leg_precision/head_precision 参数不存在 | 运行二进制或 overlay 可能仍是旧版本；核对进程实际可执行文件与安装前缀 |
| 启动后正常 Action 暂被拒绝 | 查看 /Control/motion_safety 中 head_startup_home 状态；WAITING/RUNNING 正在回零，FAILED 需排查，不会自动恢复任务 |
| ERROR_TRACKING_STOPPING / ERROR_TRACKING_STOP_UNCONFIRMED | 本次 arm_error/body_error 跟踪超限后的停止确认仍未结束；先核对新帧、实测静止和旧线程，不通过新目标强行继续 |
| status_data is empty / timestamped EtherCAT feedback unavailable | 同时核对状态字、故障码、OP/PDO、采样时间及序号；该聚合报错不能单独证明链路断开 |
| motor enable rejected / drive fault | 驱动状态与真实反馈，修改几何不能消除电机故障 |
| 工具拒绝或不动作 | 对应 tool 类型、com_port、两元素数组、串口设备状态 |
| 改 YAML 没效果 | 实际加载路径、安装文件、启动缓存、是否已重启对应进程 |
| 同一目标上次可达本次失败 | 比较起始关节、碰撞云、未参与关节和约束；不能只比较终点 |

当前 WBC IK 的迭代/权重预算是源码常量，详见 [WBC_IK_TUNING.md](WBC_IK_TUNING.md)；没有可任意 ros2 param set 的同名调参接口。规划成功、Goal 接受、上传源码、构建成功、部署安装、运行新二进制、实机到位分别需要各自证据。

### 11.1 身体运动反馈快照与采样前进

身体倾角/竖直升降的预检与首个执行周期可能读取同一帧，WBC 100 Hz 发布者也可能同时读取反馈。快照读取现使用独立的采样前进记录，不再消耗发布者的历史，避免将仍有效的重复读取误报为 `hardware sample did not advance`。快照保留原设备采集时间，不会把读缓存的时刻当成新采集。

WBC 发布、轨迹到位和停止确认仍要求真实设备帧前进；陈旧、倒序、缺失、非有限反馈和驱动故障继续拒绝。若机械臂 UDP 反馈真正停发，或设备序号一直不变，不能靠快照读取或关闭机械臂误差跟踪恢复有效反馈。

2026-11-02 验证记录：本机完整回归 14/14、x86 完整回归 18/18 通过。已直接安装到 x86 `/home/it/ylf_wbc/install` 并重启，验证 `error` 开启、rad 限制读回和关闭，最终保持 OFF。单纯跟踪超限、等待静止及旧线程退出后自动解除等分支由离线测试验证；本次未发送新的机械臂运动 Goal。SDK 反馈曾中断；在确认厂家直连 SDK 上位机完全退出后重新连接，并持续检查设备帧与安全状态。厂家直连 SDK 的连接与仅通过 ROS Action 操作的上位机应分别排查。

### 11.2 正常退出与安装生效

x86 对一次 SIGINT/SIGTERM 启动正常退出流程：关闭接单、等待旧线程结束及实测静止、提交双臂和 EtherCAT 六轴去使能、确认去使能后的新帧与连续 500 ms 静止，再释放驱动，最终输出 `shutdown=COMPLETE`。`brake_feedback=UNAVAILABLE` 表示没有独立抱闸读回证明。

`FAILED_HOLD` 保留驱动与反馈读取，重复 Ctrl+C 不绕过未完成的确认。应先查看哪一路状态、速度、反馈年龄或序号未通过，不将反复信号视为已安全退出。源码上传、编译和安装不能证明旧进程已换成新版本；还需核对 `/proc/<pid>/exe`、安装二进制哈希、实际加载的 SDK 以及最新启动日志。

### 11.3 4 ms 控制与进度发布隔离

2026-11-02 的腰腿 MoveJ 曾报 `preplanned dispatch missed 4 ms deadline at sample 1300`。原周期检查覆盖整个采样回调，该回调内还同步执行 ROS Action 反馈发布；仅凭旧错误无法区分线程唤醒延迟、目标写入耗时与发布耗时。此次移除发送循环内的同步进度发布，并保留原 4 ms 截止规则、超时、取消、反馈新鲜度和停止确认。

普通头/腰腿 MoveJ、MovePose，身体倾角/升降，固定身体姿态的腿 MoveJ，以及单/双臂 MoveJ、MoveP、MoveL 的周期循环，现在只通过无锁原子变量写入最新进度。每个活动轨迹使用独立的 **100 ms 定时线程**（上限约 10 Hz，不占用 ROS 实测反馈 executor），读取并发布最新值。没有更新时不重复发布；慢订阅者不会形成待发进度队列，也不会在恢复后集中补发。WBC 的父动作进度也移出子任务监督循环，避免发布阻塞取消及状态检查。

规划、进入执行和最终到位等阶段反馈仍可在周期循环之外发布。动作结束前停止并回收进度线程，再发布 Result，避免动作完成后继续发布旧反馈。执行失败先触发运动停止/确认，再等待正在进行的发布结束。进度发布异常会使本次动作失败；进度百分比本身不作为实测到位证明。

周期超时日志新增字段，例如：

```text
preplanned dispatch missed 4 ms deadline at sample 1300 phase=after_sample wake_late_us=60 sample_us=4200 late_us=4260
```

- `phase=before_sample`：线程唤醒时已错过截止，本点尚未发送。
- `phase=after_sample`：采样回调完成后检测到超时，本点可能已发送，后续点取消。
- `wake_late_us`：相对计划时间的唤醒延迟。
- `sample_us`：采样回调耗时；before_sample 时为 0。
- `late_us`：检测时相对计划时间的总延迟。

仍以实际时间差达到 4 ms 判定超时，不提高阈值或补发过期点。如果新版再次报错，可先用这些字段区分调度延迟和采样路径耗时，再结合 SDK/PDO、CPU 调度及系统负载继续定位。隔离发布能排除这条同步阻塞路径，不能仅凭离线测试认定此前实机错误已完全消失。

本次验证：本机编译及 **15/15** 回归、x86 真实 SDK 编译及 **19/19** 回归全部通过。新增测试在反馈发布被阻塞时仍完成 1500 个模拟 4 ms 采样，并校验原截止取消规则、最新值合并、发布异常与 Result 前线程回收。x86 已直接安装并于 2026-11-02 18:35 左右切回 `/home/it/ylf_wbc`，头部启动回零完成；进程 3774387 与安装文件哈希一致，代码段和 Build ID 与本次构建一致，两个 SDK 哈希保持不变。30 秒被动观察得到 3001 个不同的 WBC 采集时间，无倒序或重复；接单开放、`state=MONITORING`、`active_workers=0`、`arm_tracking=OFF`。此次未重新发送此前失败的腰腿/并行动作，原动作的实机复测仍待现场执行。按随后要求，新版已于 18:40 正常退出，日志确认 `DISABLED_STATIONARY_CONFIRMED` 后进入 `shutdown=COMPLETE`，未使用强制结束，最终保持运控停止。

### 11.4 其他发布路径核查

| 发布路径 | 当前实现 | 后续优化方向 |
|---|---|---|
| `/wbc/joint_states` | 独立反馈回调组，10 ms 周期；保留设备采集时间及采样前进校验 | 保留 100 Hz；任务端有反馈新鲜度要求，不能简单降频 |
| `/Control/MotorInfo`（含 TCP） | 10 ms 定时回调内更新模型、计算 FK 后可靠发布 | 进一步测量模型锁等待及发布耗时；可将观测计算与最新快照发布分开，保留采集时间 |
| `/head_joint_states`、`/leg_joint_states` | 4 ms 状态采集线程每 12 次同步发布一次，约 20.8 Hz | 可迁移到独立状态发布定时器，避免 DDS 延迟拖慢采集线程；这不是此次已隔离的指令发送线程 |
| 夹爪 Action 反馈 | 随 20 ms 串口状态轮询同步发布，最高约 50 Hz | 可独立合并最新实测夹爪状态，避免发布影响取消/超时轮询；需保留整组状态的一致性 |
| HeadPose、心跳、安全状态 | 已分别在 33 ms、1 s、100 ms 定时回调发布 | 当前无已确认的 4 ms 发送线程内发布问题；按实测耗时决定是否继续分离 |

以上是源码核查结果。此次只修改轨迹进度发布与周期超时诊断；状态话题频率、QoS、设备采集时间、SDK、URDF 和任务接口均保持原配置。

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

# Compatibility fields: keep_tcp_level must remain false for all MoveJoints.
# Enabling it is rejected. TCP level compensation is only supported by
# MovePose WHOLE_BODY_LEFT/RIGHT/BOTH (WBC); tolerance must still be valid.
bool keep_tcp_level false
float64 tcp_level_tolerance_rad 0.02

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

# Optional WBC-only TCP constraint; defaults off. Only WHOLE_BODY_LEFT/RIGHT/BOTH
# in position mode may enable it; other arm_id values are rejected when true.
# Selected arms compensate leg/waist motion throughout WBC MoveP/MoveL.
# TCP XY stays parallel to base_link XY, preserving initial TCP Z up/down.
# Start and target must already satisfy it; yaw is free, targets are not rewritten.
bool keep_tcp_level false
float64 tcp_level_tolerance_rad 0.02

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

此模板不是机器人当前运行参数快照。部分历史字段未被读取，见第 8 节。精定位与头部启动回零的新增参数分别见第 8.5、8.6 节及其独立模板；下方旧模板未列出它们时，仍使用代码默认值。

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
