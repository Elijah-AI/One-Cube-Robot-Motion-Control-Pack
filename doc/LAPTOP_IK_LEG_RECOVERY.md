# 抓取后 IK 失败的腿部升降恢复

任务：`src/task_manager/config/tasks/laptop_assembly_two_piece.yaml`，仍使用原有 `40010/50010` 注册表项。三轮抓取各配置一个独立键：`post_grasp_round_1/2/3`。

每轮双爪夹紧后先尝试原有上升 MovePose。正常成功时直接转身、调腿和放置。仅当整个 Action 以 ABORTED 返回明确的派发前 whole-body IK 失败、未观察到执行阶段且未超时，执行以下步骤：

1. `arm_id: 4`、`leg_motion_mode: 2`、`body_height_m: 0.05`：沿 `base_link +Z` 竖直升腿，保持身体 X/Y、身体朝向和上半身关节角。
2. 等待腿部 Action 的实际到位结果，再重试一次原有双臂＋腰部上升目标；视觉缓存、抓取偏移、姿态和目标基坐标不变。
3. 原有腰部绝对 −0.3 rad 转身完成后，若本轮升腿已成功，执行同模式的 `body_height_m: -0.05`。
4. 等待回降实际到位，清除本轮标记，再继续原有双关节调腿及放置。

恢复升降速度为 0.10，超时30000 ms，碰撞模式1。没有因为 IK 恢复禁用碰撞或静态 COM 约束。

恢复升腿或手臂重试失败时任务停止，避免连续累加升腿；回降失败时不进入放置。若已经升腿但重试或转身失败，任务停止在当前状态，不自动做额外下降。Task reset 会清除逻辑标记，不等于物理回降；失败后的实际姿态仍须按反馈确认。

到位超时、缺失/过期反馈、电机拒绝、取消、Goal 拒绝和已经进入执行阶段的失败不会触发升腿。分类只匹配当前 whole-body 规划器的明确 IK 错误前缀；未使用宽泛的“错误包含 IK”判断。请求代次检查防止旧任务的迟到 MovePose/MoveJoints 结果写入新任务的恢复状态。

夹爪12个动作的速度数组统一为 `[255,255]`；三次双爪夹紧为 `max_torque: [120,120]`。单爪动作仍由原 `gripper_id` 选择，不会因另一占位速度改成255而启动另一只夹爪。抓取准备开度140、放置松爪开度200及打开时的扭矩保持原样。

动作参数示例：

```yaml
- action_type: move_pose
  ik_failure_recovery:
    key: post_grasp_round_1
    lift_m: 0.05
    speed: 0.10
    timeout_ms: 30000
    enable_collision_check: 1
  params:
    # 原有双臂 + 腰部上升参数保持不变
    arm_id: 7
    whole_body_waist_only: true
    # ... 原视觉目标、速度、超时等

# 原有转身 MoveJ 成功之后
- action_type: move_joints
  if_recovery_applied: post_grasp_round_1
  params:
    arm_id: 4
    leg_motion_mode: vertical_relative
    body_height_m: -0.05
    speed: 0.10
    timeout_ms: 30000
    enable_collision_check: 1
```

任务从64个顶层动作增加为67个；原有动作之间的顺序、槽位锚点、视觉偏移和其他参数保留。新增的三个下降节点只有恢复标记存在时才下发 Goal。旧日志中的动作数组下标随插入节点发生变化，应结合本轮恢复键及动作参数定位。

验证使用真实任务 YAML 和生产行为树，在独立 localhost ROS domain 内将 MovePose/MoveJoints 的五个服务/话题明确重映射到 `/offline/ik_recovery/`。覆盖正常路径、IK恢复、到位超时、反馈故障、升腿失败、重试失败、回降失败、等待升腿结果、执行阶段失败。测试不构造机器人模块、不发送实机电机命令。

该任务必须使用匹配的新 `ylf_wbc/install` 接口、运控节点和任务节点。`ylf_wbc_test_metrics` 的独立节点/配置不属于本次修改目标。
