# WBC 末端水平约束

## 默认行为

仅 WBC `MovePose` 的 MoveP/MoveL（`arm_id=5/6/7`）可显式开启。所有 `MoveJoints`（包括独立腰腿 `arm_id=4`）设置 true 均被拒绝：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `keep_tcp_level` | `false` | 默认关闭；WBC 中设为 true，腿腰运动时由所选手臂全程补偿 TCP 水平 |
| `tcp_level_tolerance_rad` | `0.02` | 水平倾角容差，约 1.15°；有效范围 `[0.0001, 0.1]` rad |

所有运动请求默认关闭该约束。需要全程水平补偿时，在 WBC 动作中显式设置 `keep_tcp_level: true`；省略该字段与设置 false 均不启用。普通单臂/双臂 MoveP/MoveL、手臂/头部 MoveJoints 和独立腿部 MovePose 若设置 true，会在参数校验时被拒绝。任务 YAML 和 ROS Action 入口执行相同的范围检查。

以下行为仅在 `keep_tcp_level: true` 时生效。“水平”定义为 TCP 的 XY 平面平行于 `base_link` 的 XY 平面，即 TCP Z 轴保持接近 `base_link` 的 +Z 或 -Z。根据动作启动时姿态选择朝上或朝下分支，整段禁止翻面，允许绕竖直轴转向。起点及目标都必须在容差内。

计算使用 `left_arm_tool_tf` / `right_arm_tool_tf` 的实际旋转部分，不能把 SDK 法兰姿态直接当成工具 TCP 姿态。

该实现以固定 `base_link` 为参考，适用于底盘固定且水平的场景；它没有接入 IMU 重力方向，不提供底盘倾斜、行驶过程中的重力水平补偿。

## WBC 腿部运动时的全程补偿

腿腰姿态改变时，两臂安装座随躯干旋转，所选手臂同步调整关节来维持 TCP 水平。计算采用腿腰和手臂的完整运动链：

```text
R_base_tcp(q) = R_base_body(q_leg, q_waist)
             · R_body_arm_end(q_arm) · R_arm_end_tcp
```

WBC MoveP 的中间关节路径会沿同一轨迹参数加密，保留腿腰路径，并通过手臂雅可比补偿法向误差。左右臂及腿腰使用联合时间参数化，每个 4 ms 指令时刻都对应同一个全身状态。最终指令点及相邻点的关节线性插值都必须满足水平容差；不能只凭起点和终点水平就允许执行。

`arm_id: 5/6` 约束对应单臂，`arm_id: 7` 同时约束双臂。MoveL 同时保留直线几何要求。若腿腰运动导致手臂越限、碰撞或无法补偿，整个动作失败。独立腿腰 MoveJoints 不支持此开关。独立腿部 MovePose 不支持此开关。

## 动作覆盖

| 动作 | 行为 |
|---|---|
| WBC MoveP / MoveL，`arm_id=5` | 左臂 + 腿腰，仅约束左 TCP |
| WBC MoveP / MoveL，`arm_id=6` | 右臂 + 腿腰，仅约束右 TCP |
| WBC MoveP / MoveL，`arm_id=7` | 双臂 + 腿腰，同时约束左右 TCP |
| `whole_body_waist_only` | 继续固定腿 J1/J2/J3，不能通过姿态约束改变冻结关节 |
| 普通手臂 MoveJ / MoveP / MoveL / `via_poses` | 不支持，设 true 拒绝 |
| 独立腿腰 MoveJ、定躯干朝向、升降模式，`arm_id=4` | 不支持，设 true 拒绝 |
| 独立腿部 MovePose | 不支持，设 true 拒绝 |
| 头部动作 | 不支持，设 true 拒绝 |

开启约束要求位置控制。阻抗控制允许外力造成末端偏转，因此开启约束的阻抗请求会被拒绝。夹爪、吸盘开关等非位姿动作不在此路径约束接口范围内。

## 规划与执行过程

1. 从模型和启动关节状态计算真实 TCP 朝向，检查起点水平及朝上/朝下分支。
2. 对 WBC 目标检查转换到模型根坐标系后的 TCP 朝向，姿态冲突直接报错。
3. WBC MoveP 对关节路径加密，通过 Pinocchio 的 `LOCAL_WORLD_ALIGNED` 雅可比投影 TCP 法向约束。只调整被选中手臂，保留准确终点与规划得到的腿腰路径。WBC MoveL 保留原直线 IK 路径，并检查全程水平。
4. 使用已有联合时间参数化。非静止的受约束关节路径走 TOPPRA，所选手臂与腿腰共用轨迹时刻。
5. 对最终 4 ms 指令点及其关节线性插值区间再次检查水平；保持原有 URDF、速度、加速度、jerk、碰撞、COM/ZMP 检查。WBC 下发准备也复查 SDK/URDF 限位交集与 CCS J6/J7 约束。
6. 执行和到位等待期间，在共享 `MotionSafety` 中注册实测姿态检查，采用独立 Pinocchio 数据区。检查所需手臂及腿腰反馈，跨通道时间偏差上限为 20 ms。
7. 实测倾角越界、所需反馈缺失/失效或时序不满足要求时，锁存现有安全故障并拦截后续运动指令。动作退出后移除对应检查。

投影是对候选路径的局部修正，不保证在所有障碍环境或奇异位形下找到可行路径。投影、插值认证、时间参数化或最终碰撞检查失败时返回错误，不退回无约束执行。

规划检查覆盖发出的离散轨迹及其关节线性插值。实际驱动器内部插值、机械误差和停机响应仍需实机验证；反馈检查周期及通信延迟意味着软件监控不能承诺物理姿态从不短暂超差。现有软件指令拦截也不等于硬件急停或制动确认。

## YAML 使用

需要启用全程水平约束时，将参数加在 WBC `move_pose` 动作的 `params` 中，保留原有目标、速度、超时等字段：

```yaml
arm_id: 7  # 5 左臂、6 右臂、7 双臂；均与腿腰联动
keep_tcp_level: true
tcp_level_tolerance_rad: 0.02
```

不填写 `keep_tcp_level` 时默认关闭；也可以明确写为：

```yaml
keep_tcp_level: false
```

约束容差即使在关闭时也需为有效有限数值。开启约束后，WBC 目标四元数不水平或要求翻面时会失败，不会把它替换成另一个目标。

建议在启动前确认实际 TCP 定义。若机器人的当前停放姿态不水平，应在约束关闭时执行经确认的准备动作，到达水平姿态后，再为需要水平补偿的后续动作显式设置 `keep_tcp_level: true`。

## 接口与部署

本次修改了 ROS 2 Action 定义，服务端、任务客户端及其他发送这些 Action 的程序需要使用同一版本的 `com_interfaces` 并重新构建。不要混用旧消息类型与新节点。

本地需构建 `com_interfaces`、`motion_control_pkg`、`task_manager`。构建和安装不等于远端部署或运行节点已切换；本次功能开发不包含机器人上传、重启或实机运动。

## 离线验证入口

`BUILD_TESTING=ON` 时生成 `test_tcp_level_constraint` 和 `test_tcp_level_planning`：

```bash
source /opt/ros/humble/setup.bash
cmake --build build/motion_control_pkg --parallel 1
ctest --test-dir build/motion_control_pkg -R '^tcp_level_(constraint|planning)$' --output-on-failure
```

测试使用当前 URDF 和 Pinocchio，并用模拟反馈检查安全故障锁存，不启动硬件节点。覆盖默认关闭、仅 WBC 可显式开启、非 WBC 请求拒绝、无效输入、非水平起点/终点、起终水平但中间倾斜、实际工具旋转、自由转向、禁止翻面、路径锚点保持、腿腰变化时双臂补偿及实测姿态超差。

规划集成测试调用生产 TOPPRA 和 `WholeBodyModule::plan_trajectory`，覆盖 WBC 左臂、右臂和双臂 MoveP/MoveL、腰部单独参与模式、默认关闭时允许非水平起点而显式开启时拒绝，并包含腿 J1 弯曲 0.15 rad（约 8.6°）时双臂补偿的 18 自由度联合轨迹。测试目标不连接厂商驱动库；ROS 节点仅用于本机离线规划。

2026-10-20 本地验证：三个相关包串行编译、安装通过；10 组约束测试、10 组规划测试和 40 项实际 YAML 解析检查通过，覆盖默认关闭、WBC 显式开启和非 WBC 开启被拒绝。腿部补偿案例保留了 0.15 rad 的腿 J1 峰值，手臂关节最大补偿约 0.0541 rad，897 个 4 ms 指令点及点间插值通过水平检查。已核对新 Action 的 C++、Python 默认值及安装接口一致性。

本机 `task_manager` 使用现有构建缓存指定的非系统 `libmodbus`；安装后动态链接检查需将该依赖目录加入 `LD_LIBRARY_PATH`。运控节点动态链接检查通过，未启动生产运控节点，也未上传、重启或驱动机器人。

## 主要实现位置

- [Action: MoveJoints](../src/com_interfaces/action/MoveJoints.action)、[Action: MovePose](../src/com_interfaces/action/MovePose.action)。
- [接口参数校验](../src/com_interfaces/include/com_interfaces/tcp_level_goal.hpp)。
- [水平约束计算](../src/motion_control_pkg/src/motion/tcp_level_constraint.cpp)。
- [规划与监控作用域](../src/motion_control_pkg/include/motion_control_pkg/motion/tcp_level_plan.hpp)。
- [双臂模块](../src/motion_control_pkg/src/dual_arm_module.cpp)、[WBC 模块](../src/motion_control_pkg/src/whole_body_module.cpp)。
- [共享安全监控](../src/motion_control_pkg/src/execution/motion_safety.cpp)。
- [任务参数解析](../src/task_manager/src/task_config_parser.cpp)、[任务 Action 客户端](../src/task_manager/src/task_action_client_arm.cpp)。
- [约束回归测试](../src/motion_control_pkg/test/tcp_level_constraint_test.cpp)、[WBC 与时间参数化测试](../src/motion_control_pkg/test/tcp_level_planning_test.cpp)。

算法接口参考：[Pinocchio 帧雅可比说明](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/jnrh2023/template/frame.html)、[MoveIt 路径姿态约束示例](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)。本实现复用项目现有 Pinocchio，不引入 MoveIt 运行依赖。
