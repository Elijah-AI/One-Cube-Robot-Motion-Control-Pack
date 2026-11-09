# 真实关节反馈（不含底盘转角）

运控节点直接从双臂 SDK 和同进程 EtherCAT 总线读取反馈，发布：

- 话题：`/wbc/joint_states`
- 类型：`sensor_msgs/msg/JointState`
- QoS：SensorDataQoS（best effort / volatile）
- 顺序：`Joint1_L` … `Joint7_L`、`Joint1_R` … `Joint7_R`、`leg_joint1`、`leg_joint2`、`leg_joint3`、`body_joint`、`head_yaw_joint`、`head_pitch_joint`。
- `position` 为 rad，`velocity` 为 rad/s。双臂 SDK 度值转换为弧度；身体/头部 SDK 已应用零位、方向和减速比，不重复变换。
- `effort` 保留 SDK 原始力矩反馈，不承诺两个硬件接口的力矩标度完全一致。

仅在两个手臂控制器输出帧序号及 EtherCAT 成功读取序号均前进、20 个关节数据有限、状态无通信/电机故障、主机采样年龄不超过 100 ms 时发布。失败时停止新消息，不用零值或旧值刷新时间戳。手臂帧计数器正常回绕可以继续；控制器异常回退需要重启节点重新确认。

时间戳是两组主机 SDK 采样时间中的较早者，不是硬件编码器时间戳。EtherCAT SDK 未提供底层接收帧序号，因此主机成功读取不等同于已证明每次总线物理帧的新鲜度。此接口不能单独作为动态稳定性验收结论。

## 在 .80 查看

```bash
ssh neardi@172.16.21.80
source /home/neardi/ylf_wbc/setup_local.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
ros2 topic echo /wbc/joint_states sensor_msgs/msg/JointState --qos-reliability best_effort --once
ros2 topic hz /wbc/joint_states
```

若运控尚未启动，在另一个终端使用同一环境启动一次：

```bash
ros2 launch motion_control_pkg bringup.launch.py
```

不要在已有运控节点连接硬件时重复启动。发布器随运控启动，无需另开桥接节点；原 `/Control/MotorInfo`、`/head_joint_states`、`/leg_joint_states` 保留。

## 保护范围

暂不发布底盘转角，也不补零。保持 `wbc_dynamic_zmp_enabled=false`。这 20 个关节可用于静态 COM 的反馈输入，但启用静态保护仍要求水平地面、底盘静止及支撑接触条件确认，并使用模型明确的未知转角保守支撑几何。仅有话题不代表保护已启用。

## 规划期间的反馈调度

真实关节采集和 WBC 状态订阅使用独立的互斥回调组；主节点使用两线程 executor。采集路径只访问 SDK/总线，不做 Pinocchio 状态更新或 TCP FK。因此碰撞/IK 规划持有模型锁时，`/wbc/joint_states` 仍可持续发布和接收。原 MotorInfo/FK 等默认回调仍按原互斥组串行执行。

如果出现 `WBC feedback incomplete, stale or time-skewed`，新版结果会同时列出缺失/过期关节、`oldest_age_sec` 和 `skew_sec`。不要通过增大 150 ms 时效限制掩盖采集阻塞。此修复有离线调度回归测试，模拟默认回调被模型锁阻塞 1.5 s，并确认采集失败后过期检查仍然生效。

## 从实测反馈发布 TF

`bringup.launch.py` 默认同时发布标准 `/tf`、`/tf_static`，支持用 joint 名或 link 名查询两坐标系关系。
单独启动入口为 `ros2 launch motion_control_pkg robot_tf.launch.py`；可选 `joint_tf_pair` 节点将任意一对关系输出为 `TransformStamped`。
数据范围、时间戳和使用命令见 [实测关节 TF 文档](../src/motion_control_pkg/doc/ROBOT_TF.md)。
