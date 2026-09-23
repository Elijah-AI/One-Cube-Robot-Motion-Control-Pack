# 笔记本组装件：首轮两件

任务文件：`src/task_manager/config/tasks/laptop_assembly_two_piece.yaml`。
主配置已注册 `laptop_assembly_two_piece`，触发寄存器 **40010**，状态寄存器 **50010**。
参考关节姿态、夹爪开度及左右手姿态习惯来自
`/home/elijah/ros2_ws/src/task_manager/config/task_config.yaml`；参考文件本身未修改。

## 动作与槽位

| 手臂 | 抓取槽（从左到右） | 放置槽（给定偏移数组顺序） | 放置顺序 |
|---|---:|---:|---:|
| 左 | 5 | 2 | 2 |
| 右 | 7 | 4 | 1 |

1. 初始面向放置处，检测 `rack`，命名保存上表面右下角位姿。
2. 双臂同步进入参考 demo 的准备关节姿态。
3. 腰部 J4 相对转动 `-pi/2`，即当前 URDF、上身直立时俯视逆时针 90°。
4. 检测 `tray`，另存抓取处上表面中心位姿。
5. 双爪打开，双臂同步 MoveP 到第 5、7 槽上方 25 cm。
6. 双臂同步 MoveL 至上表面下方 8 cm；双爪夹紧并等待完成；同步直线退出至上方 25 cm。
7. `arm_id=6, whole_body_waist_only=true`：J4＋右臂 WBC MoveP 到放置第 4 槽上方 25 cm。
8. 腰部保持，右臂 MoveL 到表面上方 10 cm，松爪至参考开度 150，直线退出，右臂回准备姿态。
9. `arm_id=5, whole_body_waist_only=true`：J4＋左臂按相同方式放到第 2 槽并收回。

本轮共 19 个原子动作。双臂运动使用 `ARM_BOTH`、`synchronization_mode=0`，同时开始和结束；双爪使用 `GRIPPER_BOTH`。
结束时 J4 保持最后一次 WBC 求解位置，不自动回零。下一轮仍要求初始面向放置托盘。
WBC 阶段三个腿部关节不参与规划；未选中的手臂保持关节角，会随腰部一起转动。
槽口进出使用单臂/双臂 MoveL；当前腰部专用 WBC 模式仅支持 MoveP。

## 几何与姿态

所有视觉位姿必须已经是真实 `base_link` 位姿，且每次恰好一个目标。
这里不再施加相机到基座的变换。位置公式为：

```text
p_tcp = p_vision + R(q_vision) * translation
q_tcp = normalize(q_vision * rotation)
```

平移使用**原始视觉坐标轴**，与之后为夹爪对齐而右乘的四元数分开。

抓取内长以用户最终指定的 **0.530 m** 为准，14 个槽关于中心对称，中心在第 7、8 槽之间。
按两端槽贴内壁、槽宽 0.025 m 计算：

```text
pitch = (0.530 - 0.025) / 13 = 0.03884615384615385 m
x(slot) = (slot - 7.5) * pitch
```

槽序按最后一次回复“沿 X”取视觉 **+X**。因此左手第 5 槽 `x=-0.09711538461538462`，
右手第 7 槽 `x=-0.019423076923076925`；局部 `y=0`，抓取 `z=-0.08`。
首尾槽 1、14 不取件。名义 1.4 cm 挡板与 53 cm 总内长存在 2 mm 差值；
以上约定对应等厚挡板 1.384615 cm，没有同时强制两组互相矛盾的尺寸。

放置相对右下角：局部 `x=0.41/2=0.205 m`，横向沿 `+Y`：

| 槽 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---:|---:|---:|---:|---:|---:|
| y（m） | 0.0195 | 0.0445 | 0.0695 | 0.0945 | 0.124 | 0.153 |

“TCP 距上平面 10 cm”按**上方**处理，放置 `z=+0.10 m`；预备/退出高点为 `z=+0.25 m`。
高度沿视觉局部 Z，而不是基坐标竖直轴。托盘外形 58.3×44.6×32.2 cm 不重复叠加到视觉中心，
也不自动形成环境碰撞几何。

TCP X 为开合轴，TCP +Z 朝向视觉 -Z；左臂抓取姿态按现场确认的轴对应关系修正：

| 阶段 | 手臂 | TCP X 对齐 | TCP Z 对齐 | 局部四元数 xyzw |
|---|---|---|---|---|
| 抓取 | 左 | 视觉 +Y | 视觉 -Z | `[sqrt(0.5),sqrt(0.5),0,0]` |
| 抓取 | 右 | 视觉 -X | 视觉 -Z | `[0,1,0,0]` |
| 放置 | 左 | 视觉 -Y | 视觉 -Z | `[sqrt(0.5),-sqrt(0.5),0,0]` |
| 放置 | 右 | 视觉 +Y | 视觉 -Z | `[sqrt(0.5),sqrt(0.5),0,0]` |

左臂抓取的 TCP Y 对齐视觉 +X；其余姿态保留原配置。目标是配置 TCP，而非法兰。

## 新增任务参数

- `vision_detect.params.result_key`：在同一 `action_sequence` 命名保存结果。
- `move_pose.params.vision_result_key`：只读取同名结果；缺失、失败或跨序列时直接失败，不回退到最近目标。
- `vision_pose_offset.translation_frame: vision`：平移沿视觉局部轴；缺省 `base` 保持旧任务基坐标偏移行为。
- `left_vision_pose_offset`、`right_vision_pose_offset`：双臂分别计算目标，必须成对给出，支持 `arm_id=2/7`。
- 单臂 `vision_pose_offset` 同时支持 `arm_id=0/1/5/6`。使用偏移时 `bound_size` 必须全零，防止重复平移。
- 任务开始/复位清空所有序列视觉缓存；先前任务的 `rack` 不会带入下一轮。
- `move_joints.params.relative_position: true` 仅支持 `arm_id=4, joint_name: [body_joint]`。
  `position[0]` 是相对增量（rad）。任务客户端从 `/wbc/joint_states` 读取四个腰腿关节，
  生成既有四轴 LEG 绝对目标：腿部沿用反馈，J4 加增量，运控仍执行原有角度限位和碰撞检查。
  消息时间戳与本地接收时间均需在 250 ms 内，允许时钟前偏最多 50 ms；
  腰腿位置/速度需完整、唯一、有限，速度绝对值均不超过 0.01 rad/s。
  可用节点参数 `task_joint_state_topic` 或 ROS remap 指定反馈话题。
- `arm_id=4, joint_name: [body_joint], relative_position: false` 表示腰部绝对角目标，
  同样读取新鲜、静止的完整腰腿反馈并保持三个腿部关节。任务最开始以 `position: [0.0]`
  回腰部零位，成功后才检测 rack；需要配套更新后的 task_manager。

以后更换本轮槽位，可修改任务文件的 `left_pick_x`、`right_pick_x`、`left_place_y`、`right_place_y` 四个 YAML 锚点；
所有高点、低点、退出点自动复用相应数值。抓取 x 由上式计算，放置 y 由上表选择。

## 验证范围

本次只在本地编写、编译和使用模拟 Action 服务离线验证，不触发 Modbus，不启动机器人驱动。
默认速度为接近 0.2、WBC/转腰 0.15、直线 0.1，碰撞模式为 1（自碰撞）。
新 YAML 必须配套本次 task_manager，旧客户端不支持命名结果、双臂视觉偏移或相对转腰。
离线通过不代表实测视觉下所有目标可达，也不验证所持板件和托盘的碰撞包络或实际夹持效果。
