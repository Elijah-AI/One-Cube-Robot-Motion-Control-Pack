# HHworkspace 三分支合并与离线验证

日期：2026-09-06（Asia/Shanghai）

工作区：`/home/elijah/ylfrobot`

目标分支：`HHworkspace`

## 合并基准

| 分支 | 原始提交 | 主要功能 |
| --- | --- | --- |
| HHworkspace | `8d7130339b251591b402ad00ee5536454c3e6b5b` | MoveL 连续 IK / 稳定 TOPPRA、双夹爪并发、现有任务配置 |
| develop | `4c517684ae016d665b95ca6c778c3acabda4f5a3` | MovePose 环境点云碰撞规划、头部位姿发布、SyncedRobotVision 点云字段、RViz 关节 GUI |
| modify_whole_body | `02cf2b633109c979285960a20239267bae328e6a` | base_link 碰撞模型更新、取消额外躯干俯仰限制、全身轨迹准备/下发分离、算法说明及示例 |

合并提交：`3e083f9`（develop）、`26fb148`（modify_whole_body）。三个原始提交均为目标分支祖先，保留各分支历史。唯一文本冲突是 `dual_arm_module.hpp` 中的头文件引入，已同时保留夹爪工作线程和预规划关节命令支持。

Gitee fetch 多次返回 TLS handshake 错误，无法确认远端是否另有更新。本次结果绑定上述本地提交；未执行 push。

## 兼容处理

- 保留 HHworkspace 的任务 YAML、Action 定义、MoveL 稳定求解和夹爪并发功能；现有 TCP 偏移仍为左侧 `0.159 m`、右侧 `0.2205 m`，未采用 develop 示例中的零偏移。
- 单臂 MovePose（`arm_id=0/1`）支持直接传入 `env_point_cloud`，也可用参数 `collision_pointcloud_topic` 接收 `SyncedRobotVision.camera_rgbd_pointcloud`。参数默认空字符串，兼容原有仅自碰撞检查；设为 `/fusion/synced_robot_vision` 后等待该话题，缺云/超时会失败。点云必须同时启用 `enable_collision_check`。
- 点云按模型帧和规划起始关节状态转换到模型根坐标系；验证 XYZ 字段、数据尺寸、有限值及坐标系。无效云抛出错误，构建失败不会悄悄删除旧障碍或继续运动。每次规划的临时障碍在规划后移除，共享碰撞场景串行访问；单臂 MoveP/MoveL 的最终 4 ms 样本均做碰撞检查。
- `/Control/HeadPose` 保留 `head_pitch_link` 相对规划坐标系的 FK，正常发布补上时间戳，FK 失败不发布未初始化矩阵。移除 PCL 与运控代码的角度转换宏冲突。
- 全身两路准备完成后再确定共同起点；双臂和腿部使用同一个 4 ms 调度函数。准备数据单次消费，下发失败/异常传播取消，阻止其他通道继续发送；准备期间取消/超时会阻止后续位置下发。准备阶段包含控制模式设置/电机使能，不能视为纯计算。
- `modify_whole_body` 的额外 30° 躯干俯仰限制移除行为保留。`body_pitch_limit_rad` 仅为源码兼容保留，当前不施加限制；URDF 关节限位和 CCS J6/J7 约束仍保留。
- `display.launch.py` 增加 `use_joint_state_gui` 开关，默认关闭，离线查看模型时可显式启用。
- 将实际生产算法抽为 `motion_planning_core`，新增测试链接同一份算法代码；测试不链接 MarvinSDK/Kine。补齐 PCL、libmodbus 和显示相关依赖，libmodbus 支持从隔离前缀查找。
- 修复安全监控原有 `Heartbeat/heartbeat.hpp` 拼写错误，匹配生成接口 `HeartBeat/heart_beat.hpp`。实际 YAML 回归测试改为核对当前配置中的每个动作、顺序、参数和并行分组，替代旧的固定组数断言；任务 YAML 未修改。

## 构建与测试结果

环境：本机 x86_64、Ubuntu 22.04、ROS 2 Humble、GCC 11.4、Pinocchio 4.0.0、Coal 3.0.3、PCL 1.12；运控和任务管理采用 Release/O2。单任务构建，底层显式 `cmake --build ... --parallel 1`。

成功配置、构建和安装 7 个源码包：`com_interfaces`、`onecube_robot_description`、`motion_control_pkg`、`task_manager`、`robot_time_sync_node`、`safety_monitor`、`camera_driver`。内嵌第三方 toppra 使用仓库提供的 x64 库，未作为独立工作区包重建。

| 测试包 | GTest 用例 | 错误/失败/跳过 |
| --- | ---: | --- |
| com_interfaces | 6 | 0 / 0 / 0 |
| motion_control_pkg | 36 | 0 / 0 / 0 |
| task_manager | 46 | 0 / 0 / 0 |
| 总计 | 88 | 0 / 0 / 0 |

`colcon test-result --test-result-base build --verbose`：

```text
Summary: 88 tests, 0 errors, 0 failures, 0 skipped
```

新增 15 个用例，位于：

- `motion_control_pkg/test/test_merged_planning.cpp`（9 项）：头部/腿部状态变化后的头部 FK、点云坐标转换、非法云拒绝、Octree 参数和非有限值、障碍阻断 OMPL/替换/移除、左/右/双臂全身 IK、放宽俯仰后的约束、异常 IK 输入、同步视觉消息 CDR 序列化。
- `motion_control_pkg/test/test_preplanned_dispatch.cpp`（6 项）：三种全身轨迹拆分、非法样本、双线程共同起点和 4 ms 样本时刻、启动前取消、子通道失败和异常传播。

原有 MoveL 采样、TOPPRA、夹爪并发、SDK 命令门、坐标变换、Action 契约和 task_manager 回归均通过。真实 YAML 通过 `TASK_MANAGER_REAL_CONFIG_PATH` 显式加载，未跳过。RViz launch 构造检查通过，未启动 RViz 或节点。

证据：`build/<package>/{configure,build,install,ctest}.log`；GTest XML 在 `build/<package>/test_results/<package>/`。算法测试的动态依赖检查没有 MarvinSDK/Kine，也没有缺失库。

## 复现

已具备系统开发依赖时，在仓库根目录执行：

```bash
bash tools/test_hhworkspace_merge.sh
```

本机 libmodbus 缺失，测试使用从 Ubuntu 官方仓库下载并校验的软件包，仅解包到临时目录：

```bash
MERGE_DEPENDENCY_PREFIX=/tmp/ylfrobot-merge-deps.hI8xkf/extracted/usr \
  bash tools/test_hhworkspace_merge.sh
```

`libmodbus-dev_3.1.6-2_amd64.deb` SHA-256：`a242c783dd227e60ab2151da5b4c5d220432a54581a87a967fc2248792ffe51c`。

`libmodbus5_3.1.6-2_amd64.deb` SHA-256：`eff9f5473c079f0f9df2d62ac9e6084749388e831fdd25d1c3c3578a66e3db2a`。

`MERGE_BUILD_ROOT` / `MERGE_INSTALL_ROOT` 可指定独立的绝对构建/安装目录。脚本不下载依赖，不启动机器人节点；测试使用 localhost 和 ROS domain 218。`rosdep` 缓存未初始化，因此本次使用 CMake 依赖发现和实际编译验证依赖。

## 验证边界

- 本次无机器人连接、动作下发或部署，未做 ARM64 编译、驱动运行和物理验收。调度测试验证调用时刻和取消逻辑，不代表真机 4 ms 实时性、到位、负载稳定性或两通道物理同步精度。
- 环境点云新增链路针对普通单臂 MovePose/MoveL；双臂与全身模式保留已有的模型自碰撞逻辑。本仓库 `robot_time_sync_node` 目前不生成点云；话题方式需要视觉侧实际填充该字段，也可由调用方通过 Action 直接提供。
- 点云帧必须存在于当前模型中；按规划起始关节状态变换，并未实现历史时刻 TF/运动补偿。场景应为视觉侧处理后的环境障碍点云。
- `SyncedRobotVision.msg` 新增字段后，相关生产者和消费者必须按同一接口版本重新编译；源码合并不保证新旧 ROS 消息二进制互通。机器人使用的 `/opt/onecube` 配置和标定需在部署时另行核实。
- 全身动作的成功仍不能代替物理到位判定；示例轨迹、放宽俯仰限制后的稳定性及实际碰撞模型适配尚需真机验收。
- 工作区一致性静态检查为 `0 errors, 44 warnings, 1 info`，包含历史依赖/安装规则提示；没有将这些提示当作已完成的运行验证。原有未跟踪副本和 Windows Zone.Identifier 文件保留，未纳入提交。
