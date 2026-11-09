# 实测关节 TF 树与任意两坐标系变换

运控使用 `/wbc/joint_states` 的实测位置驱动 `robot_state_publisher`，发布标准 ROS 2 TF 树。

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | 随关节位置变化的变换，默认最高 50 Hz |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | URDF 固定连接和关节名别名，transient local |
| `/Control/joint_transform` | `geometry_msgs/msg/TransformStamped` | 可选：指定两个坐标系的合成变换，默认 20 Hz |

## 启动

正常使用 `bringup.launch.py` 时默认启动 TF 发布器。已有其他节点负责同一机器人 TF 时，可用
`publish_robot_tf:=false` 关闭这份发布器，避免重复发布相同的父子关系。

已有运控进程、只需单独启动 TF 时，在相同 ROS_DOMAIN_ID 和网络配置下运行：

```bash
source /opt/ros/humble/setup.bash
source /home/it/ylf_wbc/install/setup.bash
ros2 launch motion_control_pkg robot_tf.launch.py
```

该入口只消费关节状态，不连接 SDK、不初始化电机。已有 TF 发布器时无需重复启动。

## 查询任意两个关节

标准 TF 使用 URDF **link 名**作为坐标系。本功能也为每个 **joint 名**增加同名坐标系，
与该关节的 child link 重合，表示施加关节转动/位移之后的坐标系。
例如 `Joint7_L` 与 `Link7_L` 重合，`leg_joint2` 与 `leg_link2` 重合。
原始 URDF 和规划模型不修改；这些别名仅增加在 TF 发布器的内存模型中。

```bash
# 右臂第 7 关节坐标系在左臂第 1 关节坐标系中的变换
ros2 run tf2_ros tf2_echo Joint1_L Joint7_R

# 头部俯仰关节坐标系在第二腿关节坐标系中的变换
ros2 run tf2_ros tf2_echo leg_joint2 head_pitch_joint

# 也可直接使用原始 link 名
ros2 run tf2_ros tf2_echo base_link Link7_L
```

不要只在单条 `/tf` 消息里寻找指定关节对。话题发布树的各条边，跨分支关系由 TF2
缓存 `/tf` 和 `/tf_static` 后合成。客户端调用
`buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())` 获取最新公共时刻的关系。

## 直接从一个 topic 读取指定关节对

TF 树已运行时，另开一个终端：

```bash
ros2 run motion_control_pkg joint_tf_pair --ros-args \
  -p target_frame:=Joint1_L \
  -p source_frame:=Joint7_R
```

读取结果：

```bash
ros2 topic echo /Control/joint_transform geometry_msgs/msg/TransformStamped --once
```

也可在单独启动 TF 树时一起启动这个输出：

```bash
ros2 launch motion_control_pkg robot_tf.launch.py \
  target_frame:=leg_joint2 source_frame:=head_pitch_joint
```

输出的 `header.frame_id` 是 `target_frame`，`child_frame_id` 是 `source_frame`，
表示 `T_target_source`，满足 `p_target = R * p_source + t`。
平移单位是米，旋转为四元数 `x, y, z, w`，时间戳保留 TF 的采集时间。
需要同时输出多对时，启动多个 `joint_tf_pair`，分别设置节点名和输出 remap：

```bash
ros2 run motion_control_pkg joint_tf_pair --ros-args \
  -r __node:=head_leg_tf \
  -p target_frame:=leg_joint2 -p source_frame:=head_pitch_joint \
  -r /Control/joint_transform:=/Control/head_leg_transform
```

关节对参数只在该小节点启动时设置；改变关节对只需重启该节点，或由客户端直接查询 TF2。

## 数据范围和时间

- 实测输入包含双臂 14 个关节、腰腿 4 个关节、头部 2 个关节。这 20 个关节及其固定附件之间可查询任意关系。
- 底盘转向、轮子和夹爪当前没有该采集流的反馈。TF 不为这些自由度补零；依赖缺失动态边的查询会不可用。
- 上游反馈异常时 `/wbc/joint_states` 停止更新，动态 TF 也停止更新；固定 TF 仍可查询。
- TF2 的“最新”查询可能返回缓存中的旧变换。`joint_tf_pair` 默认要求动态路径的最新公共时间不超过 0.15 秒，过期或未来时间不发布，不修改原时间戳。静态路径时间戳为零时可持续输出。
- `joint_tf_pair` 输出为 volatile，订阅者应使用时间戳和接收超时判断数据是否仍然有效。
- 模型内固定夹具或相机 frame 来自 URDF。运行时工具 TCP 参数没有自动变成 TF frame；`Joint7_L/R` 指 URDF 子连杆坐标系。

## 单独 TF 启动参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `urdf_path` | 已安装的 `onecube_robot_description.urdf` | 与运控相同的模型 |
| `joint_state_topic` | `/wbc/joint_states` | 实测关节状态输入 |
| `publish_frequency` | `50.0` | 动态 TF 发布频率上限，Hz |
| `publish_joint_frames` | `true` | 是否提供 joint 名别名 |
| `frame_prefix` | 空 | 多机器人 TF frame 前缀；查询时需使用加前缀后的名字 |
| `target_frame` / `source_frame` | 均为空 | 同时设置时启动指定变换输出 |
| `transform_topic` | `/Control/joint_transform` | 指定变换输出话题 |
| `pair_publish_frequency` | `20.0` | 指定变换输出频率，Hz |
| `pair_max_age_sec` | `0.15` | 动态变换允许的最大年龄，秒 |

`joint_tf_pair` 独立运行时，使用参数 `publish_frequency` 和 `max_age_sec` 设置后两项。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/motion_control_pkg/test/robot_tf_test.py src/motion_control_pkg
```

测试使用独立的本机 DDS 域（默认 187，可通过 `ROBOT_TF_TEST_DOMAIN_ID` 指定），
启动真实 RSP 和 TF2，输入已知的 20 关节位置，以独立的 URDF 矩阵计算核对跨臂、腿头变换及逆变换。
同时检查无反馈不补零、采样停止后不刷新动态 TF、过期后停止合成话题输出，以及新反馈恢复后的输出。
若要验证当前已安装的入口、脚本和模型，先 source 工作区的 `install/setup.bash`，再在上述测试命令前设置
`ROBOT_TF_TEST_USE_INSTALLED=1`。默认模式直接测试源码，不依赖工作区已完成安装。
