# 腿腰和头部精确到位

2026-10-22，本地工作区 `/home/elijah/ylf_wbc`。

## 启用范围

| 路径 | 最终关节误差 | 低速接近段 |
|---|---|---|
| 普通腿腰 MoveJ（含 body_joint/J4） | ≤ 0.002 rad | 开启 |
| 头部 MoveJ（yaw、pitch） | ≤ 0.002 rad | 开启 |
| 腿腰 MovePose、双腿关节姿态保持、身体倾斜/升降 | ≤ 0.002 rad | 保留原约束路径，不增加独立补偿段 |
| 全身协调 MovePose 的腿腰 | ≤ 0.002 rad | 保留同步轨迹，不独立重放腿腰 |
| 双臂 | 原来的 0.01 rad | 不变 |

腿腰、头部要求速度绝对值 ≤ 0.01 rad/s、至少 5 帧有效递增反馈，并连续满足条件 200 ms。这是编码器反馈的关节误差，不代表末端绝对定位精度。

## 普通 MoveJ 行为

先规划完整路径，再执行。最大单关节位移大于 0.03 rad 时，先移动至距目标最大单关节差 0.03 rad 的接近点，以零规划速度/加速度结束第一段；等待反馈进入接近点 0.01 rad 范围并稳定后，执行一次低速末段，到达原始目标。小于等于 0.03 rad 的动作整段低速。

末段速度取原速度乘 0.2、0.03 rad/s 上限及 URDF 约束的共同限制；加速度、jerk 同样限制。两段均用本地 Ruckig 状态到状态规划，检查实际 4 ms 离散命令导数，原有碰撞检查继续执行。接近点等待后重新检查剩余超时预算。

这会增加运动时间，原有 action 超时仍然生效。不能仅凭第一段发送完成宣布成功；最终必须通过实测反馈判定。

不会在失败后自动重新启动动作，不向目标叠加位置偏置，不清除安全故障、不自动使能。等待期间总线维持最后目标。取消、超时、失效反馈、命令拒绝均终止后续发送，沿用现有安全停止处理。

静止后若误差仍超限，且连续 1.5 s 没有至少 0.0002 rad 的改善，则报告 stalled，并输出误差最大关节索引（链内从 0 开始）、目标、实测及容差。持续静差需要结合驱动位置环、负载、摩擦与反馈分辨率检查；重复发送相同目标不能保证消除静差。

## 参数

启动参数前缀为 `leg_precision` 与 `head_precision`，相互独立；全身协调路径读取同一套腿腰完成判据。双臂不读取这些参数。

| 后缀 | 默认值 | 含义 |
|---|---|---|
| goal_tolerance_rad | 0.002 | 最终单关节误差上限 |
| stopped_velocity_rad_s | 0.01 | 稳定判据速度上限 |
| stable_time_sec | 0.2 | 连续稳定时长 |
| stall_time_sec | 1.5 | 停止改善的判定时长，0 禁用该诊断 |
| minimum_progress_rad | 0.0002 | 有效改善量，必须小于最终容差 |
| completion_timeout_sec | 5.0 | 独立腿腰/头部每次等待上限，仍受 action 总超时限制 |
| approach_enabled | true | 普通 MoveJ 低速接近功能 |
| approach_distance_rad | 0.03 | 最大关节差定义的接近距离 |
| approach_speed_scale | 0.2 | 末段速度缩放 |
| approach_velocity_rad_s | 0.03 | 末段速度上限 |
| approach_acceleration_rad_s2 | 0.2 | 末段加速度上限 |
| approach_jerk_rad_s3 | 0.5 | 末段 jerk 上限 |

全身协调动作保留原来的 3 s 完成等待上限；其中腿腰使用新的容差、稳定和停滞判据。`approach_enabled=false` 只关闭分段规划，不放宽最终精度。参数为启动时只读，修改后需重启节点。示例覆盖文件：[precision_arrival.yaml](../src/motion_control_pkg/config/precision_arrival.yaml)。代码默认值已启用上述策略；未自动修改机器人 `/opt/onecube` 的外部配置。

## 运动学库参考

- [Pinocchio CLIK 示例](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_d_inverse_kinematics.html)：迭代误差阈值衡量模型中的位姿残差，不能代替硬件到位判定。MoveJ 已有目标关节角，无需靠收紧 IK 阈值解决跟踪滞后。
- [Ruckig 教程](https://docs.ruckig.com/tutorial.html)：规划位置、速度、加速度及其边界约束；轨迹完成与硬件实际稳定应分开判定。
- [ROS joint_trajectory_controller 参数](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)：分别设置路径误差、目标误差、停止速度和目标等待时间。本实现借鉴这种分离方式，没有修改现有路径跟踪安全阈值。

## 离线验证与使用边界

`test_precision_arrival` 使用真实 Ruckig 轨迹与模拟反馈，覆盖腿腰和头部速度约束、离散导数、原始目标不变、滞后到位、200 ms 稳定、旧双臂容差保持、反馈停更/倒退/非有限值、静差停滞、接近点取消、发送拒绝及超时预算。模拟反馈用于验证控制逻辑，不能证明真机可达到 0.002 rad。

验证命令：

```bash
source /home/elijah/ylf_wbc/setup_local.bash
cmake --build /home/elijah/ylf_wbc/build/motion_control_pkg --target test_precision_arrival motion_control_node --parallel 1
ctest --test-dir /home/elijah/ylf_wbc/build/motion_control_pkg --output-on-failure
```

本次只修改本地源码并进行离线编译测试；未安装到机器人、未重启控制节点、未发送真实运动指令。真机验收应记录实际位置/速度、反馈时间戳、末段耗时和最终误差，在可控低速条件下确认后再提高前段速度。

本次验证结果：`motion_control_node` 编译成功；CTest 的 `precision_arrival`、`drive_control`、`tcp_level_constraint`、`tcp_level_planning` 全部通过（4/4）。精确到位测试内部包含 9 组场景。日志保存在 `/home/elijah/precision_arrival_20261022/build_final.log` 和 `/home/elijah/precision_arrival_20261022/ctest.log`。
