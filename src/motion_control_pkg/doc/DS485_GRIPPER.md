# DS485 共用 SDK 夹爪

目标工作区为 `/home/it/ylf_wbc`。夹爪包 `src/ds485_driver` 集成独立工程 `93f0b749` 的恒定 PWM，以及现场已部署的双侧位置控制器，通过导出的 `ds485_driver::ds485_control` 静态库直接调用。
公开接口：`ds485_driver/gripper_controller.h`、`ds485_driver/gripper_config.h`。
完整交接说明：`/home/it/ylf_wbc/src/ds485_driver/doc/MOTION_CPP_INTEGRATION.md`。
`motion_control_node` 统一建立 Marvin SDK 连接；夹爪模块只 Attach，退出先停止并 join
夹爪线程，最后由运控释放 SDK。只加载运控自带的一份 SDK，不运行独立 `ds485_node`。

## 接口选择与调度

自动任务使用 `/Control/control_gripper`，类型 `com_interfaces/action/ControlGripper`。
沿用现有 Goal/Feedback/Result，不修改 com_interfaces 的消息定义。
`block_until_complete` 必须为 true；数组均至少包含 `[左,右]` 两项。
到位或检测到阻挡均成功；取消、超时、故障或停止失败返回失败信息。

手动调试保留原 topic：

| Topic | 类型 |
|---|---|
| `/ds485/gripper/command` | `ds485_interfaces/msg/GripperCommand` |
| `/ds485/gripper/stop` | `ds485_interfaces/msg/GripperStop` |
| `/ds485/gripper/result` | `ds485_interfaces/msg/GripperResult` |
| `/ds485/state` | `ds485_interfaces/msg/ServoState` |

两个入口使用同一执行器，每侧仅一个工作线程访问总线。
同侧 topic 新命令替换旧 topic 命令，旧命令先尝试锁舵并返回 PREEMPTED；
Action 与 topic 同侧互斥，忙时拒绝新请求，左右侧可以独立运行。
双侧请求统一检查资源后接纳；Action 任一侧失败会取消另一侧尚未结束的命令。
Topic 的双侧结果仍按 side=0/1 分别发布。Action 的结果通过 Action 返回，不混入 topic 命令编号。
停止 topic 可终止 Action 或 topic。Action cancel 只取消本次 Goal 的命令，不误停后续命令。
未知/丢失运动应答、取消、超时和退出会尝试锁舵；不能确认停止时保留失败状态并拒绝新动作。
出现保护会结束当前命令。四类可恢复异常（电压、电流、温度、堵转）消失后，启用自动恢复的侧可由下一条新指令触发固件复核。硬件或协议故障仍拒绝运动。停止和取消不会触发恢复。

位置策略使用上述 Action 和话题。恒定 PWM 策略通过同一运控进程中的 `/ds485/servo/set_torque` 服务控制输出。
`ds485_ws` 仍可用于单独调试；使用 Marvin 透传时必须与运控择一运行。

## 配置

`bringup.launch.py` 读取 `/opt/onecube/motion_control/config/motion_control_config.yaml`。
当前机器人标定：左开/闭 **0.2° / 98.1°**，右 **57.9° / 167.6°**。
所有夹爪参数集中在 `/opt/onecube/motion_control/config/ds485_gripper.yaml`，
主运控配置仅包含 `gripper_type: ds485` 和该文件的 `ds485_gripper_config` 路径。
独立文件沿用 `ds485_node.ros__parameters.left/right` 格式，包含角度、通道、ID、速度、PWM、轮询和超时。
修改后重启运控；驱动代码改动则先重新编译 ds485_driver，再链接运控。
速度/力矩 0 采用默认值，当前为舵机上限速度与 PWM=1000；示例使用 50/100 比例值。
所有参数的范围、单位及不适用于集成模式的字段见上述完整说明。

左右侧各自读取 `protection`。本机左侧为 5–30 V、60 °C（回差 5 °C）、6000 mA 持续 200 ms、堵转 PWM 600 持续 3000 ms。右侧各阈值为 `-1`，保留舵机现值。两侧 `auto_recover` 均启用。启动或重新连通时按差异同步并读回保护设置，运行中的命令不会自动重试。新出现的保护位会记录温度、电压、电流和 PWM。

## 切换和使用

1. 在独立驱动的终端 Ctrl+C，确认 `ds485_node` 正常退出、UDP 4730 释放。
2. 新终端加载正式运控工作空间，沿用现场运控的 ROS domain / RMW 配置，命令端与任务端保持一致。集成模块不强制修改 DDS 环境。
3. 启动运控。此操作会执行原有机器人启动流程，需要由现场操作人员进行。

```bash
source /opt/ros/humble/setup.bash
source /home/it/ylf_wbc/install/setup.bash
ros2 launch motion_control_pkg bringup.launch.py
```

下列指令会实际控制夹爪，应单条执行。发送命令的终端也要加载上面的环境。

Action 示例（左侧闭合）：

```bash
ros2 action send_goal --feedback /Control/control_gripper com_interfaces/action/ControlGripper \
  "{task_id: 'left_grip', gripper_id: 0, position: [255,0], speed: [50,50], max_torque: [100,100], com_port: [2,2], block_until_complete: true, timeout_ms: 10000}"
```

已有脚本也可使用：`ros2 run motion_control_pkg ds485_gripper close left 100 50 10000`。

Topic 示例（左侧张开；side 0 左、1 右、2 两侧）：

```bash
ros2 topic pub --once -w 1 /ds485/gripper/command ds485_interfaces/msg/GripperCommand \
  "{id: 1, side: 0, position: 0, torque: 100, speed: 50, timeout_ms: 10000}"
```

查看 topic 结果：`ros2 topic echo /ds485/gripper/result`。
停止两侧：

```bash
ros2 topic pub --once /ds485/gripper/stop ds485_interfaces/msg/GripperStop "{side: 2}"
```

## 离线验证

独立驱动 CTest 包含协议/舵机/夹爪、共用连接执行控制及独立配置解析测试。
运控 `ds485_ros_bridge` 覆盖真实 Action/topic 通信和取消，并使用 SDK Spy 验证只 Attach、不 Connect；
Fake 总线和 localhost/domain 187 不触及硬件。`ds485_offline` 保留旧协议回归。
在 `ylf_wbc` 中依次构建 `ds485_interfaces`、`ds485_driver` 和 `motion_control_pkg`，机器人使用 `cmake --build ... --parallel 1`。
软件安装与现场的物理停止、夹持和开合验收分开记录。

## 运控内的恒定 PWM

GX12-02 使用的夹爪源码取自 `ds485_ws` 提交 `93f0b749841bf4442b379718820a6fe9e6dd5df8`。在运控的标准 bringup 上使用 `ds485_control_strategy:=torque` 可启用 `/ds485/servo/set_torque`。默认 `position` 沿用本机的 `ControlGripper` 与开合标定。两种策略在启动时择一，运控进程使用已连接的 MarvinSDK。恒定 PWM 的调用、制动确认和寄存器地址见[运控指令与参数](../../../doc/MOTION_COMMANDS_AND_PARAMETERS.md#511-恒定-pwm)和[驱动协议说明](../../ds485_driver/doc/constant-torque.md)。
