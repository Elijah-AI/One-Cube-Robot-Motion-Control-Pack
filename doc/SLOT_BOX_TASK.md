# 插槽盒任务：右臂一号槽、左臂三号槽

任务名 `slot_box_right_1_left_3`，已在
[`task_config.yaml`](../src/task_manager/config/task_config.yaml) 注册。
动作文件为 [`slot_box_right_1_left_3.yaml`](../src/task_manager/config/tasks/slot_box_right_1_left_3.yaml)。
触发寄存器 `40009`，状态寄存器 `50009`，与命令示例的 `40006～40008` 分开。

每次任务先调用 `/vision/detection_pose_cloud` 获取盒子右下角位姿，随后双臂到预抓点。
右臂依次 MoveP 到一号槽上方 25 cm、10 cm，打开右夹爪并等待完成，
MoveP 回到 25 cm，然后 MoveJ 回到右预抓点。
右臂返回成功后，左臂对三号槽执行相同步骤并回到左预抓点。
整轮仅检测一次，全部动作在同一条顺序序列中；任一步失败即停止后续步骤。

预抓关节角直接复用 `task_config.yaml` 引用的 `tasks/move_box.yaml`
首个并行组中的左右臂记录值。这里没有调用通用 `go_pick_pose` 任务，也没有加入腰腿动作。
六个笛卡尔动作均为 `enable_move_line: false`，即 MoveP；上下点之间不承诺直线路径。
手臂速度比例统一设为 `0.2`，超时 45 秒。夹爪沿用原任务串口 `[2,2]`、
活动通道速度 `255`、最大扭矩 `60`，位置 `0` 表示全开，超时 10 秒。

位置计算使用基坐标轴，单位米：

```text
p_tcp = p_corner + [0.204, slot_y, height]
height = 0.25 / 0.10
q_tcp = normalize(q_corner * q_offset)  # 局部旋转，右乘；四元数顺序 xyzw
```

| 槽号 | x 偏移 | y 偏移 | 本任务使用 |
| --- | --- | --- | --- |
| 1 | 0.204 | 0.0170 | 右臂 |
| 2 | 0.204 | 0.0435 | — |
| 3 | 0.204 | 0.0665 | 左臂 |
| 4 | 0.204 | 0.0915 | — |
| 5 | 0.204 | 0.1225 | — |
| 6 | 0.204 | 0.1495 | — |

右臂 `q_offset=[0.707107,0.707107,0,0]`，
左臂 `q_offset=[0.707107,-0.707107,0,0]`。
输入和偏置四元数均会归一化；平移不随角点姿态旋转。
每一步以缓存中的原始角点计算新目标，不覆盖原始数据，不累计偏移。

本任务增加的参数示例：

```yaml
vision_pose_offset:
  translation: [0.204, 0.017, 0.25]
  rotation: [0.707107, 0.707107, 0, 0]
```

该参数只允许单臂、`use_config_pose: false`、非空 `frame_id`，
并要求 `bound_size` 全零，防止运控再次叠加偏移。
`target_pose` 是旧解析器字段兼容占位，缺少同序列视觉结果时不会使用占位值运动。
运动目标仍按现有运控 TCP 定义解释；这里不修改机械臂 TCP 标定参数。

**视觉接入条件**

`vision_detect` 配置的目标名为 `slot_box_corner`，含义是插槽盒右下角。
视觉服务应返回唯一的角点 `detected_targets[0].pose`，位置为米，
实际坐标系为 `base_link`，姿态为该角点的原始姿态。
任务检查目标数量、每个目标自身的 `pose.header.frame_id`、有限数值和有效四元数。
没有目标、有多个目标、坐标系错误等情况会在预抓动作前失败。

根据你确认的“视觉位姿都基于基坐标系”，视觉服务已将旧的 `gripper_link`
标签改正为 `base_link`，结果总头和目标自身坐标系保持一致；没有对已有基坐标数值再次变换。
`expected_frame_id` 不能触发坐标变换。旧视觉二进制若仍返回 `gripper_link`，
本任务会拒绝该结果，因此需要更新视觉服务或接入符合上述约定的服务。
当前服务的 `target_obj_name` 是结果标签，不会按名称切换到新的角点检测算法；
这里沿用已有插槽检测管线，按你给定的语义将结果解释为盒子右下角。

**构建与配置生效**

新参数需要本次更新后的 `task_manager` 二进制；只替换 YAML 不会添加姿态偏置能力。
在已具备依赖、与当前 Action 定义一致的 ROS 2 工作区中：

```bash
cd /home/elijah/ylfrobot
source /opt/ros/humble/setup.bash
# 先 source 本机已构建且与当前源码一致的 com_interfaces overlay。
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --packages-select task_manager --executor sequential --parallel-workers 1
# ARM64 目标机上另行构建视觉服务（需要仓库 NPU/RKNN 依赖）：
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --packages-select vision_detection_pose_cloud --executor sequential --parallel-workers 1
```

节点启动时读取一次 `config_path`。完成构建并停止旧任务节点后，
在相同 ROS_DOMAIN_ID 环境中 source 本次 install，再显式加载源码配置：

```bash
source /home/elijah/ylfrobot/install/setup.bash
ros2 launch task_manager task_manager.launch.py \
  config_path:=/home/elijah/ylfrobot/src/task_manager/config/task_config.yaml
```

另一个终端确认：

```bash
ros2 param get /task_manager_node config_path
```

当前回合仅完成本地配置、源码和离线验证，没有启动或重启机器人节点。
本机验证构建位于 `/home/elijah/ylf_merge_20260917/build/task_manager`，
不代表机器人端已经安装本次更新。
任务节点完整编译通过，25 项单元测试通过，其中新增 6 项覆盖偏置计算、
四元数乘法顺序、坐标系拒绝、预抓点复用和完整动作顺序。
视觉服务修改已通过 C++ 对象编译；本机是 x86，仓库 NPU 库为 ARM64，
视觉可执行文件的链接和运行仍需在目标机完成。

**触发命令（会执行机械臂和夹爪动作）**

当更新后的任务节点、视觉与运控已准备好时，在任务节点主机使用：

```bash
# 0 -> 1 触发一次；远程调用时把 127.0.0.1 换为任务节点主机地址。
mbpoll -m tcp -p 12000 -a 1 -t 4 -r 9 -1 127.0.0.1 0
mbpoll -m tcp -p 12000 -a 1 -t 4 -r 9 -1 127.0.0.1 1

# 1=运行中，2=成功，3=失败。
mbpoll -m tcp -p 12000 -a 1 -t 4 -r 10009 -c 1 -1 127.0.0.1
```

碰撞模式默认 `1`（自碰撞）；`0` 为关闭，`2` 为自碰撞加外部碰撞。
视觉角点结果不含环境点云，单纯设为 `2` 不会自动获得插槽盒的外部碰撞几何。
