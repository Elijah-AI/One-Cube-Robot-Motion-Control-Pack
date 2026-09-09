# 验收记录

日期：2026-07-30（Asia/Shanghai）
平台：ROS 2 Humble，真机双臂，控制器 `192.168.1.190`
工作区：`/home/hh/ros2_ylf_demo`

## 整理结果

工作区仅包含 4 个源码包：

1. `com_interfaces`
2. `onecube_robot_description`
3. `motion_control_pkg`
4. `task_manager`

没有复制旧工作区的 `build*`、`install*`、`log*`、`.git`、缓存、rosbag 或 `/tmp/dual_arm_merge_build.*` 部署。安装后的 4 个包均由 `ros2 pkg prefix` 解析到本工作区的 `install/`。

## 编译与自动测试

- 干净编译：4/4 包成功
- 自动测试：88 项，0 error，0 failure，0 skipped
- `task_manager`：5 个测试程序、46 项断言全部通过
- 合并后的接口契约已验证：`MovePose`/`MoveJoints` 均定义 `ARM_BOTH=2`
- 独立单臂并发已验证：左右臂资源可同时占用、分别完成和分别释放

## `updateChainJointsAngle` 更新频率审计

双臂的真实 Marvin SDK 状态由 `joint_feedback_timer_` 每 `250 ms` 读取一次，随后 `updateArmModelState()` 对左右臂分别调用 `updateChainJointsAngle()`。因此常规运动过程中的真实关节模型更新频率是 **4 Hz**。

需要区分两条时序：

- 轨迹命令发送周期是 `4000 us`，即 **250 Hz**。
- `updateChainJointsAngle()` 的常规真机反馈更新是 **4 Hz**，并不是随每个 4 ms 指令点更新。
- MovePose 到位等待阶段会反复读取 SDK 并更新模型：单臂循环间隔 `20 ms`（最高约 **50 Hz**），双臂循环使用 `4 ms` 周期（最高约 **250 Hz**）；这些是动作末段的到位/FK 检查，不代表整个运动过程始终以该频率更新。

结论：函数在运动时会持续获得真实反馈更新，但常规频率为 4 Hz；若需求是“整个轨迹执行期间严格 250 Hz 实时更新 Pinocchio 模型”，当前实现尚未达到该语义。

## 真机任务验收

通过本机 `192.168.1.123:12000` 向任务寄存器写入一次触发：

```bash
mbpoll 192.168.1.123 -p 12000 -a 1 -t 4 -r 1 -1 1 -o 2
```

任务配置：两个并行 `MoveJoints` 分支，左臂 `arm_id=0`、右臂 `arm_id=1`，两臂 Joint1 从约 `1.0 rad` 运动到 `0.8 rad`，其余关节为 0，速度比例 `0.2`，超时 `10000 ms`，碰撞检查关闭。

结果：

- 左臂完成时间：约 `1.21059 s`；Action 报告最大误差 `0.000103739 rad`
- 右臂完成时间：约 `1.21030 s`；Action 报告最大误差 `0.000082567 rad`
- 最终反馈：左 Joint1=`0.79999981 rad`，右 Joint1=`0.80000014 rad`
- 触发寄存器恢复为 0
- 状态寄存器为 2（成功）
- `task_manager` FSM 返回 IDLE，`errFlag=0`
- 新工作区的 motion control 与 task manager 进程在验收后保持运行

现场人工区域限制和急停条件仍是碰撞检查关闭时的必要安全前提。
