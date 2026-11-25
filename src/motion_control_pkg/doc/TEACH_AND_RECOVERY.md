# 天机示教开关与整机清错恢复

> `.44` 吸盘交付：基于 2026-11-13 最终版本（含提交前反馈重采样修补），保留本机双臂完成判定。
> 本文 `.18/.23` 的硬件验证、TCP 和动力学数值不代表 `.44`；本机标定及 `/opt` 配置保持原样。
> 本轮只交付源码和离线验证结果；运行版本、安装版本与源码状态分别记录在本次交付记录中。

## 适用机器与本次操作边界

SDK 位置限位合并版将双臂范围统一到 SDK 配置，并保留下述示教与清错流程。
限位表、初始化顺序与合并验证见 [SDK 限位与示教合并版](SDK_LIMITS_AND_TEACH_RELEASE.md)。

2026-11-13 版本：运控位于 `it@192.168.10.18:/home/it/ylf_wbc`，
配套任务节点位于 `neardi@192.168.10.23:/home/neardi/ros2_ws`。
两机原有程序和接口已安装。示教停稳要求为每阶段最长 5 秒，
窗口位移上限 0.05 rad，反馈速度上限 0.03 rad/s；仍要求至少 5 个有效递增帧连续跨越 100 ms。
SDK 限位合并版已在 `.18` 启动核实 14 轴范围，并成功完成整机清错保持使能和左臂示教开启。
首次合并验收的关闭请求在提交前新帧检查被保护性拒绝；新增有界重采样修补已上传源码并通过
`.18` 的 13 组离线回归。随后机器人网络中断，最后修补的安装和真机复测仍待恢复连接后确认，
详见上方合并说明的“验证范围”。最后一次确认退出为 2026-11-13 20:02:01，`shutdown=COMPLETE`。
本次交付日志及备份位于 `/home/it/tianji_teach_recovery_20261113/final_combined/`。
两个 service 均由运控节点提供，调用它们不需要启动 task_manager，开关命令不变。

本轮按用户授权执行清错和示教开关请求，没有执行普通运动任务或人工拖动；具体成功/失败结果见上述验收记录。
正常启动运控包含驱动初始化，并可能触发头部自动回零；示教正常退出及
`enable_after_clear: true` 都会发送保持当前实测位置的关节目标。
`enable_after_clear: false` 也会发送下使能和驱动复位命令，因此这些操作不是只读查询。

## 终端准备与服务检查

建议在 `.18` 上直接使用。下面统一采用现场进程核实的 ROS 域 `43` 和 Fast DDS：

```bash
ssh it@192.168.10.18
cd /home/it/ylf_wbc
source /opt/ros/humble/setup.bash
source /opt/onecube/driver_sdk/setup.sh
source install/setup.bash
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 以下只查看已安装接口，不要求运控已启动
ros2 interface show com_interfaces/srv/SetTeachMode
ros2 interface show com_interfaces/srv/RecoverRobot
```

两机 `.bashrc` 原有设置是域 `42`、`rmw_cyclonedds_cpp`；本次未修改这些文件。
**服务端与命令终端必须使用相同 ROS 域和兼容 DDS 配置。** 若以后运控改用其他设置，
命令终端应跟随实际运控设置；不要同时混用本示例的 43 和旧文档的 42。

在允许驱动初始化/头部回零的正常运行阶段，确认没有已有运控实例后，
可在上述终端启动运控并保持终端运行：

```bash
# 会启动真实驱动，可能执行头部回零，不是只读检查
ros2 launch motion_control_pkg bringup.launch.py
```

远程退出时应核对运控 PID，向运控发送正常 SIGINT，并确认
`/Control/shutdown_state` 到达 `shutdown=COMPLETE` 后再启动。
本次实测只向 launch 父进程发送 SIGINT 没有转发给运控，不等同于终端 Ctrl+C；不要强杀。

另开终端，重复 `source` 和上述三项 `export`，再检查：

```bash
ros2 service list -t
ros2 service type /Control/set_teach_mode
ros2 service type /Control/recover_robot
ros2 param get /motion_control_ns/motion_control teach_mode_available
ros2 topic echo /Control/motion_safety std_msgs/msg/String --once
```

服务类型应分别是 `com_interfaces/srv/SetTeachMode` 和 `com_interfaces/srv/RecoverRobot`。
`waiting for service to become available` 表示服务尚不可见，不表示操作已成功；先检查
运控是否运行、是否加载新安装、ROS 域/DDS 是否一致。不要因服务不可见而重复启动运控。

若从 `.23` 调用，在其终端将工作区换为 `/home/neardi/ros2_ws`，source 本机
`install/setup.bash`，并使用与 `.18` 一致的上述 ROS 设置；service 名和请求体完全相同。
示教期间不要运行会持续提交动作的任务链；当前 task_manager 没有新增自动示教、
自动恢复的 YAML `action_type` 或 Modbus 入口。

## 参数与 SDK 依据

`bringup.launch.py` 从 `/opt/onecube/motion_control/config/motion_control_config.yaml`
加载参数。示教复用 `left/right_arm_tool_tf`、`left/right_tool_dynamics` 和
`left/right_joint_impedance_*`。工具几何 TF 的 m/quaternion 转换为 SDK 的 mm/deg；
工具动力学数组直接传给 SDK，运行时不会再转换坐标系、单位或惯量参考点。
工具动力学顺序为 `[M, MCP_X, MCP_Y, MCP_Z, I_XX, I_XY, I_XZ, I_YY, I_YZ, I_ZZ]`。
质量为 kg；质心为 SDK 法兰坐标系下的 mm；惯量为法兰原点、法兰坐标轴下的 kg·m²。
执行器零位、方向、减速比、EtherCAT XML 仍使用机器原配置。

实现依据用户指定的 `D:\一立方机器人\TJ_FX_ROBOT_CONTRL_SDK-master` 中
`DEMO_C++/showcase_drag_JointImpedance_save_data.cpp`、
`contrlSDK/MarvinSDK.cpp::SetJointDrag` 和 `OnSetDragSpace_A/B`：
`OnSetTargetState_*(3)` → `OnSetImpType_*(1)` → `OnSetDragSpace_*(1)`。
所有缓存型命令都经过现有 SDK 事务锁；工具参数先写入并回读。
拖动开关打开后，需要按住机械臂物理末端按钮才能拖动。
示教退出用 `OnSetDragSpace_*(0)`；未采用示例中的 `OnSetImpType_*(0)`，
因为该 SDK 实现只接受阻抗类型 1～3。

### 当前 `.18` 完整夹爪的动力学转换

2026-11-13 经用户确认，以 `.18` **当前安装 URDF 的完整夹爪**替换 `/opt` 中两组
旧动力学参数。数据源为：

```text
/home/it/ylf_wbc/install/onecube_robot_description/share/onecube_robot_description/urdf/onecube_robot_description.urdf
SHA256: 986833f67d04ca82189207599e4844ff54e13b292607fe071072efda083c3686
/home/it/ylf_wbc/install/motion_control_pkg/share/motion_control_pkg/config/ccs_m6_40.MvKDCfg
SHA256: 9544fb24b3249e67639d5392282c69e383ad905c138d6cfcb15f0bf09c6a58d9
```

左右分别合并 `gripper_base_l/r_link`（1.5 kg）和 `gripper_l/r_link1`、
`gripper_l/r_link2`（各 0.05 kg）。两手指按 URDF 的限位 `lower=upper=0` 固定在零位。
这是 URDF 的标称空夹爪数据，不是称重或实机辨识结果，也不包含被抓工件。

SDK 使用法兰参考系 F。该机型最后一行 MDH 为 `[90°, 0 mm, 95 mm, 90°]`；
URDF 的 `Link7` 到夹爪基座 G 为 `xyz=[0.00036386,-0.095,0] m`、
`rpy=[1.5708,-1.5708,0] rad`。用 `F_T_G = inverse(Link7_T_F) * Link7_T_G`
计入约 0.364 mm 横向偏移，并保留 URDF 角度的小数精度。
不能将 Link7、夹爪基座和 TCP 三个原点混为一谈，也不能仅交换旧质心的 Y/Z 分量。

对每个部件，先把质心和质心惯量旋转、平移到 F，再用平行轴定理合并：

```text
M = sum(m_i)
c_F = sum(m_i * c_F_i) / M
I_F = sum(R_F_i * I_C_i * transpose(R_F_i)
          + m_i * (dot(c_F_i,c_F_i) * Identity - c_F_i * transpose(c_F_i)))
```

计算中长度用 m，输出质心再乘 1000 变为 mm；惯量保持 kg·m²。
SDK 依据为 `c++_doc_contrl.md` 的 `OnSetTool_A/B` 数组顺序、
`c++_doc_kine.md` 第 11 节的法兰参考要求，以及
`kinematicsSDK/LoadIdenPub.cpp` 的 `OnCalLoadDynamic_7_MarvinCCS/SRS`：
代码将质心惯量平移到法兰原点，并仅将质心乘 1000 输出。

| `/opt` 参数结果 | 左夹爪 | 右夹爪 |
|---|---:|---:|
| 总质量，kg | 1.6 | 1.6 |
| 法兰质心 XYZ，mm（显示到 6 位） | [-0.000003, -0.225135, 74.964878] | [0.000003, -0.503136, 74.964877] |

当前 TCP 几何外参仍是 `[0,0,0.2695,0,0,0,1]`，与法兰同轴。
质心主要沿法兰正 Z；如果改为以 **SDK TCP 原点**表达，质心 Z 约为
`74.964878 - 269.5 = -194.535122 mm`，因为质心在 TCP 后方。
写入 `tool_dynamics` 的是上表的法兰坐标，不能改填这个 TCP 坐标。

实际写入的数组（不要用此片段覆盖整个 YAML）：

```yaml
left_tool_dynamics: [1.6, -3.1804505207468e-06, -0.2251347660843, 74.964878180955,
  0.013447777149923, -7.3864267876414e-07, 7.5007498700515e-10,
  0.012963947776465, 3.4726496247917e-05, 0.0015061741660997]
right_tool_dynamics: [1.6, 3.1824734175877e-06, -0.50313595665816, 74.964877159799,
  0.013448100841166, -7.3863896864932e-07, -7.5582358527139e-10,
  0.012963947644841, 5.2643182870959e-05, 0.0015064979889653]
```

已用 Pinocchio 独立合并惯量、检查整体质心惯量的物理一致性，并在本地用厂商
纯运动学库进行 64 组计算核对 MDH 法兰约定。没有连接驱动或下发这些计算姿态。
旧数组的原始坐标系无法追溯；源码默认值和通用模板仍是历史值，不能拿来覆盖本机 `/opt`。
末端参数转换阶段只替换 `/opt` 中 `left_tool_dynamics`、`right_tool_dynamics` 两项，其余字节保持不变。
参数在运控启动时读取；本轮恢复修补未改 `/opt`，已验证启动和恢复，未做实体拖动验证。

离线复算方法（只生成报告，不连接机器人，也不会修改 `/opt`）：

```bash
cd /home/it/ylf_wbc
python3 src/motion_control_pkg/tools/convert_gripper_dynamics.py \
  --urdf install/onecube_robot_description/share/onecube_robot_description/urdf/onecube_robot_description.urdf \
  --sdk-config install/motion_control_pkg/share/motion_control_pkg/config/ccs_m6_40.MvKDCfg \
  --output /tmp/gripper_tool_dynamics.json
```

依赖 Python 3 和 NumPy；输出文件须不存在。脚本拒绝未核实的法兰 DH、与 DH 不匹配的
手臂模型、未锁定的手指关节及无效惯量。换工具、改变手指模型或增添工件后应按实际装配重新计算。
本次原始模型、配置备份、计算明细和校验结果保存在 `.18` 的
`/home/it/tianji_teach_recovery_20261113/tool_dynamics_conversion/`。

## 示教 service

2026-11-13 过渡态修复补丁：开启/退出示教及恢复使能期间，仅对本次选中的手臂，
在有期限、可取消的切换事务内接受预期的 `103`（转扭矩）或 `101`（转位置）反馈。
这些状态仍不能用于普通运动；最终必须收到目标状态的多帧停稳确认。
SDK 错误码、过期反馈、错误的过渡方向、有效位置限位越界和切换超时仍会使操作失败。
示教开启、正常关闭及其中配置写入的每个停稳阶段，最长等待 **5 秒**。
每个关节的窗口位移（相对本次连续窗口首帧）上限为 **0.05 rad**，
反馈速度上限为 **0.03 rad/s**；安全监控还用位置差分核对速度，阈值同为 0.03 rad/s。
仍须至少 5 个递增设备帧且连续跨越 100 ms，反馈新鲜、SDK `low_speed=1`、
按钮释放、状态/拖动标记正确、无错误且满足有效关节限位。
条件不满足会重新累计连续窗口，但不重置该阶段 5 秒的总超时。
提交前新读的一帧若失去低速、速度、按钮或状态资格，同样在剩余时间内重新等待完整窗口；
不重复发送 SDK 命令。反馈会记录 `precommand_resamples` 及最后一帧的状态，持续异常仍超时拒绝。
有效位置限位指双臂采用 SDK 范围、腿/腰/头采用原有 URDF 范围。
切入扭矩模式后原有固定 1 秒等待仍保留；完整 service 含多个阶段，耗时可能超过 5 秒。
这些容差只用于示教切换；清错恢复、故障复位、保护下使能、正常运动准入和退出运控
保持原有较严格的停稳条件。service 名称、参数和开关命令不变，示教默认关闭。
等待失败的消息包含通道、活动标记、静止帧数、静止时间和实测速度。
退出示教的位置保持快照在等待停稳之后重新读取，并再次检查按钮、速度和有效位置范围。

2026-11-13 退出顺序修补：正常关闭分为两个独立 SDK 命令批次，先仅发送 `drag=0`，
等待新鲜、递增、连续停稳的反馈确认拖动退出，再次等待安全监控停稳并重新采集实测位置，
最后发送位置保持目标及位置模式请求，确认 `state=1, drag=0`。第一阶段未确认、按钮重新按下、
仍在移动或请求被取消时，不发送第二阶段的位置目标和模式命令。全过程保持普通运动封锁。
故障锁存下的关闭仍只退出拖动并下使能，不会发送位置保持目标或自动恢复使能。

关闭失败现在明确区分 `teach OFF drag-exit confirmation` 和
`teach OFF position confirmation`，同时报告预期状态、实际状态、拖动值、错误码、按钮、
低速标志、最大实测速度、帧号和反馈年龄；不再把正常切回位置模式的超时统称为
`disabled/standstill`。每阶段等待有超时和取消检查，不自动重发失败的模式切换。
此次修补的分阶段退出已进行离线模拟验证，真实机械臂退出结果仍需现场验收。

每次启动默认 **OFF**。`teach_mode_available` 默认为 `true`，可在上述 YAML
的 `ros__parameters` 中设为 `false`，禁止开启；关闭操作始终可请求。
该参数在节点启动时读取，实际开关由 service 控制。

| 开关 | 默认值 | 作用 |
|---|---|---|
| 节点参数 `teach_mode_available` | `true` | 是否允许收到开启请求；它本身不会开启示教 |
| service 请求 `enable` | `false` | `true` 开启，`false` 退出；进程启动时实际示教状态始终为 OFF |
| service 请求 `target` | `2` | `0` 左臂 A，`1` 右臂 B，`2` 双臂；不是 MoveJoints 的完整 arm_id 枚举 |

如要禁止现场操作员开启示教，将下面一行合并到 `/opt` 原 YAML 的现有节点参数内，
下一次正常启动生效；不要用该小片段覆盖整个配置文件：

```yaml
/motion_control_ns/motion_control:
  ros__parameters:
    teach_mode_available: false
```

它是启动只读参数，不能用 `ros2 param set` 热切换。保持 `true` 也仍需显式 service
请求才能进入示教；本次 `/opt` 仅更新了上节所列的两组工具动力学参数。

实际示教顺序（仅在以后允许开启阻抗时使用）：

1. 停止任务提交，等待运动、夹爪/吸盘动作和启动流程全部完成；反馈健康且停稳。
   正常进入示教前不需要先调用 `stop_motion`，该调用会锁存故障并使开启请求被拒绝。
2. 松开所选机械臂的末端拖动按钮，发送一次开启请求。
3. 检查 `success: true`、`feedback_confirmed: true`，且 `active_mask` 含所选臂。
   此后按住物理末端按钮拖动，松开按钮后停稳。
4. 用相同 `target` 发送关闭请求。正常退出保持拖动后的实测位置；关闭全部示教臂后
   `active_mask: 0`，再次确认 `/Control/motion_safety` 健康，再由上层发起新的任务。

```bash
# 双臂开启；此命令会进入阻抗示教，本次禁止执行
ros2 service call /Control/set_teach_mode com_interfaces/srv/SetTeachMode '{target: 2, enable: true}'

# 先松开按钮并停稳，再关闭。正常退出保持当前实测位置。
ros2 service call /Control/set_teach_mode com_interfaces/srv/SetTeachMode '{target: 2, enable: false}'
```

单臂只将两条命令的 `target` 同时改成 `0` 或 `1`。切换所选臂时先退出当前已开启的
示教臂，再开启新的选择；不要重复发送开启请求来更新选择。
重复 `enable: true` 不会退出示教，也不会再次下发开启命令；响应会说明当前
`active_mask`，若故障已锁存会同时提示需要显式恢复。现在包括重复开启、忙和参数拒绝在内的
每次请求都会记录 `Teach switch` 日志，包含 target、active_mask、command_issued 和反馈确认结果。
`active_mask` 非零也可能是切换失败后的保护标记；不能仅凭该值认为当前仍处于阻抗。

开启要求：无运动/工具任务、启动完成、反馈健康并停稳、拖动按钮已释放。
收到成功响应后再按按钮拖动。开启期间拒绝普通运动和夹爪/吸盘 Action；
SDK 层也拒绝轨迹写入。`/Control/motion_safety` 中的 `teaching_mask` 为
0=关闭、1=左臂、2=右臂、3=双臂；非零也可能表示切换结果尚未确认。

成功响应必须收到多帧递增反馈，确认扭矩状态、关节阻抗和拖动类型。
切换不完整、超时、反馈失效或越过模型限制时保留运动封锁并请求手臂下使能。
故障锁存期间关闭示教只退出拖动并下使能；重新使能通过整机恢复完成。
这些反馈确认不代表机械制动器或 STO 的反馈确认。

| 响应字段 | 如何判断 |
|---|---|
| `success` | 本次切换调用及反馈均确认成功 |
| `command_issued` | 已进入命令配置/下发阶段；不代表切换已确认 |
| `feedback_confirmed` | 多帧更新反馈确认所请求的模式和拖动状态 |
| `active_mask` | 0 关闭，1 左臂，2 右臂，3 双臂；失败时非零也可能表示结果不确定、仍封锁普通运动 |
| `message` | 拒绝原因、SDK 状态或保护性下使能结果；失败时保存此字段排查 |

## 整机恢复 service

它是整机维护入口，一次覆盖双臂、腰腿和头部，无需逐部件调用。
使用前停止任务提交，等待所有已接受/规划/执行的运动及工具任务释放资源。
服务内部会安装软件封锁，不要求先调用 `/Control/reset_motion_fault`。

| 请求 | 结果与适用场景 |
|---|---|
| `enable_after_clear: false` | 只下使能、停稳、清错；保留软件故障锁存，清错成功也不能立即发运动 |
| `enable_after_clear: true` | 清错后在实测位置重新使能，检查反馈并解除软件锁存，允许后续新 Goal |

请求默认值为 `true`，建议始终显式填写，避免用空请求误触发恢复使能。

```bash
# 只清错；保持下使能和软件故障锁存。
ros2 service call /Control/recover_robot com_interfaces/srv/RecoverRobot '{enable_after_clear: false}'

# 清错并恢复到可接收新任务，会下发实测位置保持目标
ros2 service call /Control/recover_robot com_interfaces/srv/RecoverRobot '{enable_after_clear: true}'
```

以上是两种选择，不要求连着执行。先做“只清错”后，若以后允许恢复使能，再显式请求
`true`；第二次调用会重新执行完整检查和清错流程。

流程：封锁动作入口 → 各组下使能并确认停稳 → 双臂、腰腿、头部逐组清错
→ 可选地在实测位置使能 → 反馈确认 → 解除软件故障锁存。
调用要求已接受/规划/执行的运动和工具任务退出；忙时拒绝，交由调用方停止任务后重新请求。

- 双臂：`OnClearErr_A/B`，确认控制器和 14 个伺服错误码，恢复时清除拖动设置。
- 腰腿：逻辑 ID 29、28、27、26，对应 EtherCAT alias 30、29、28、27。
- 头部：逻辑 ID 31、30，对应 alias 32、31。
- EtherCAT：由原总线线程发送 `enabled=-1` 复位脉冲，再回到 `enabled=0`；
  以时间戳和 PDO 序号确认，禁止用重复缓存读数认定成功。
- 重新使能前检查正常有效位置范围、静止和反馈时效，保持固定实测位置；
  EtherCAT 使能要求 600 ms 连续有效反馈。任何部件未确认，均不返回整机就绪。
- 双臂使能前仍要求速度不超过 0.01 rad/s；发送实测位置保持目标后，在有界等待
  内允许使能过渡的短暂速度收敛，但持续检查有效位置限位及原有速度限值、反馈时效，
  且相对保持目标的偏移不得超过 0.01 rad。只有位置模式、拖动关闭、按钮释放、
  低速标志和原有连续停稳窗口全部通过才确认使能；持续运动仍超时失败。
- 所有部件使能确认后，安全监控和实测状态发布器建立新的反馈帧基准，解决
  下使能期间帧号回绕/重置后被旧帧号误拒绝的问题。此时仍保留运动封锁；
  仅接收新基准建立之后采集的反馈，按原有时效、有效关节限位和速度检查重新累计
  至少 5 帧、跨越至少 100 ms 的连续停稳反馈。最多等待 2 秒，满足后才解除锁存。
  冻结/倒退帧、下使能状态或故障反馈仍不能解除锁存；失败仍对所有组执行下使能。
- `physical standstill not confirmed` 现在附带未通过通道的 `fresh`、`sequence`、
  `stationary_frames`、`measured_speed` 和错误原因，便于区分反馈问题和未停稳。
- 清错阶段某组失败仍尝试其余组清错，之后不使能；使能/软件恢复失败时请求所有组下使能。
- 恢复和回退阶段若 SDK `EStop` 未取得停稳/下使能确认，追加一次检查返回值的
  伺服关闭事务，并重新等待新反馈确认。此补偿不下发关节目标、不使能、不清错；
  补偿仍失败时明确返回未确认，不把 SDK 函数已调用视为下使能成功。
- `/Control/stop_motion`、急停维护请求或进程退出可取消恢复；恢复不会重试旧轨迹。

响应分别提供 `arms_cleared`、`lower_body_cleared`、`head_cleared`、
`drives_enabled`、`motion_ready`。`success=true` 且 `motion_ready=true`
表示本次完整恢复已确认；清错但保持下使能时 `motion_ready=false`。
本接口不执行操作系统重启或控制器进程重启。

预期成功响应（省略具体 `message`）：

| 字段 | 仅清错成功 | 清错并恢复成功 |
|---|---|---|
| `success` | `true` | `true` |
| `arms_cleared` / `lower_body_cleared` / `head_cleared` | 均为 `true` | 均为 `true` |
| `drives_enabled` | `false` | `true` |
| `motion_ready` | `false` | `true` |

完整恢复后再读取一次 `/Control/motion_safety`，应看到 `state=MONITORING`、
`teaching_mask=0`、`maintenance_active=0` 及健康反馈。如果 `success: false`，
`drives_enabled: false` 只表示没有确认整机恢复使能，不是所有电机均已下使能的证明；
结合 `message` 中各组结果与新反馈判断。不要用软件复位绕过失败步骤，也不要自动重试旧轨迹。

原 `/Control/reset_motion_fault` 仍只复位软件锁存；原 `/Control/drive_control`
新增 `HEAD=4`，其他 target 和 operation 数值不变。

## 常见拒绝与处理

| 返回/现象 | 下一步 |
|---|---|
| 找不到新接口类型 | 在当前终端 source 对应机器的正式 install；用 `ros2 interface show` 检查 com_interfaces |
| service 不可见 | 检查运控进程、实际运行版本、ROS 域和 DDS；安装文件更新不代表现有进程已加载补丁 |
| `teach_mode_available is false` | 能力开关禁止开启；如以后需要启用，修改 `/opt` 原 YAML 后在正常运行阶段重启 |
| `maintenance requires idle...` / `another maintenance request is active` | 停止继续提交任务，等待启动、运动/工具资源和前一维护操作退出；不要并发调用维护服务 |
| 示教按钮、静止或健康检查不通过 | 松开所选末端按钮，等待停稳；按 message 排查反馈/故障，不能靠反复开启跳过检查 |
| 切换超时、`active_mask` 非零且 `success: false` | 保留封锁，读取状态和保护性下使能结果；按故障处理，不能把它当作成功开启或关闭 |
| `teach OFF drag-exit confirmation` | 拖动退出未确认；按消息检查 `drag`、按钮、速度、帧号和反馈年龄，后续位置切换未执行 |
| `teach OFF position confirmation` | 拖动退出已确认，但切回位置模式未确认；保留封锁并检查保护下使能结果，不反复发送开启请求 |
| 某个 `*_cleared` 为 false | 查对应手臂、腰腿或头部的错误码/通讯/停稳情况；本次不允许进入恢复使能阶段 |
| 有效位置限位越界或反馈过期 | 排查模型、标定、实测位置及通讯；恢复服务不会通过放宽限位或使用旧反馈来强行使能 |

`/Control/stop_motion` 可以取消正在进行的恢复并锁存软件停止；这是软件控制入口，
不等同硬件急停。故障处理中应检查停止反馈确认，而不是只看调用已经返回。

需要显式手臂下使能时，使用下面的维护入口；它不是正常示教开关的准备步骤。
`target=0/1/2` 分别为左臂、右臂、双臂。执行前停止推动、松开拖动按钮并确保负载有支撑。

```bash
# 双臂下使能并等待实测停稳确认；不发送关节位置目标
ros2 service call /Control/drive_control com_interfaces/srv/DriveControl '{target: 2, operation: 0}'
```

必须检查 `success: true` 和 `feedback_confirmed: true`。正常关闭示教则是切回位置保持，
会发送当前实测关节角。保护下使能已确认后，常规运动监控因不接受下使能状态 `0`，仍可能显示
`STOP_UNCONFIRMED`；应结合本次维护响应与时间戳判断，不能靠软件复位绕过真实驱动恢复。

## 任务节点和部署

`com_interfaces` 必须在运控机器和任务机器同步构建，尤其本版本的 MovePose/MoveJoints
同时携带精定位、TCP 水平及刚性抓持字段。task_manager 保留机器人现有的
TCP 反馈等待、条件恢复及任务 YAML，转发最新动作字段。
不会在任务失败后自动调用整机恢复，也不会在任务链中自动开启示教。
此次过渡态补丁没有修改接口或任务节点，无需为此重新构建 `.23` 的 task_manager。

离线测试覆盖示教状态确认、按钮和反馈检查、退出实测位置保持、恢复部分失败、
取消、有效位置限位越界和软件封锁。部署后的实体拖动手感、带负载保持和真实驱动故障
恢复需要现场按上述入口验收；构建/安装不会自动发起这些操作。
