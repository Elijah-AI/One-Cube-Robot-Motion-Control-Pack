# 全身 MoveP：仅腰部 J4 与手臂

每条 `/Control/move_pose` 指令可设置 `whole_body_waist_only: true`。
默认 `false`，保留原全身模式；省略字段的任务 YAML 也保持原行为。

| 参数 | 要求 |
|---|---|
| `arm_id` | `5` 左臂、`6` 右臂、`7` 双臂 |
| `enable_move_line` | `false`（MoveP） |
| `whole_body_waist_only` | `true` |

新模式的活动自由度为所选手臂加 `body_joint`（腰部 J4）：单臂 8 自由度，双臂 15 自由度。
`leg_joint1/2/3` 固定为规划起点的反馈角度，头部及未选手臂保持原关节角度。
腿部不参与 IK、随机种子重试或 OMPL 避障搜索；Ruckig/TOPPRA 及动态参考处理后的轨迹逐点检查腿部角度不变。
目标超出该模式可达范围或无法通过碰撞、COM/ZMP 等已启用检查时，动作失败，不自动切回允许腿部运动的模式。

下肢驱动仍使用原四关节同步位置接口：J1/J2/J3 收到恒定位置保持命令，J4 收到腰部轨迹。
这不是腿部断使能模式，仍需原有下肢资源互斥、有效反馈及执行保护。
碰撞模式 `0/1/2` 和 COM/ZMP 开关保持原有独立语义；设置本字段不会自动开启平衡保护。

## Action 示例

在 [wbc_left_waist_only_goal.yaml](examples/wbc_left_waist_only_goal.yaml) 中替换为现场核实的 TCP 目标后，使用已有 MovePose 客户端发送。
该文件只是消息格式示例，没有验证其中目标的现场可达性。
`arm_id=6` 使用 `target_pose`；`arm_id=7` 使用 `left_target_pose`、`right_target_pose` 及对应的 `left_bound_size`、`right_bound_size`、`synchronization_mode`。

## task_manager YAML

在现有 `move_pose` 动作的 `params` 中加入字段即可，普通动作和 `move_compound` 内的 MovePose 子动作均支持：

```yaml
arm_id: 5
enable_move_line: false
whole_body_waist_only: true
```

其余目标位姿、速度、超时等参数照原动作配置填写。普通手臂 `arm_id=0/1/2`、其他部件或 MoveL 配合 `true` 会被明确拒绝；要用原模式，省略字段或填写 `false`。

## 编译与接口版本

本次新增了 MovePose Goal 字段，客户端与服务端必须同时更新生成接口及其依赖包，不能混用旧版二进制：

```bash
cd /path/to/ylf_wbc
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --executor sequential --parallel-workers 1 \
  --packages-up-to motion_control_pkg task_manager \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

其他独立 MovePose 客户端也需重新生成接口并重编译。已有节点不会因修改源码自动更新，需要在部署后使用匹配的安装空间启动。
