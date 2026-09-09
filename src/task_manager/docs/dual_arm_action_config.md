# 双臂 Action 配置

`task_manager` 继续使用 `/Control/move_pose` 和 `/Control/move_joints`。
`arm_id: "0"`、`"1"` 保持原单臂字段；`arm_id: "2"` 使用合并 Action 的显式左右目标。

## 双臂关节目标

```yaml
- action_type: move_joints
  params:
    arm_id: "2"
    left_joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L]
    left_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    right_joint_name: [Joint1_R, Joint2_R, Joint3_R, Joint4_R, Joint5_R, Joint6_R, Joint7_R]
    right_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    synchronization_mode: 0
    enable_collision_check: false
    speed: 0.05
    timeout_ms: 10000
```

## 双臂 TCP 目标

位姿为 `[x, y, z, qx, qy, qz, qw]`，位置单位为米。目标是绝对 TCP 位姿，
不是相对增量。

```yaml
- action_type: move_pose
  params:
    arm_id: "2"
    frame_id: base_link
    left_target_pose: [0.2061, 1.0841, 2.0618, -0.6428, 0.0, 0.0, 0.7661]
    right_target_pose: [0.2061, -1.0842, 2.0611, 0.6428, 0.0, 0.0, 0.7661]
    left_bound_size: [0.0, 0.0, 0.0]
    right_bound_size: [0.0, 0.0, 0.0]
    synchronization_mode: 0
    enable_collision_check: false
    use_force_control: false
    speed: 0.03
    hand_action_type: []
    timeout_ms: 10000
```

当前双臂服务端仅支持 `synchronization_mode: 0`，且双臂位姿模式不支持力控、
灵巧手动作、非零工具包围盒或环境点云；客户端会在发送前拒绝这些组合。

旧的 `action_type: move_arm` 仍作为 `move_pose` 的兼容别名，但新配置应使用
`move_pose`。
