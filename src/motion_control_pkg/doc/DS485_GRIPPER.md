# DS485 自制夹爪接入（GX12-02）

适用于德晟第五版协议，经 Marvin 末端 485 透传：左=A，右=B。沿用 `/Control/control_gripper`，`com_port: [2,2]` 为 COM1，3 为 COM2。`gripper_type` 默认为 `jodell`，选择 `ds485` 后使用本驱动。

`bringup.launch.py` 实际读取 `/opt/onecube/motion_control/config/motion_control_config.yaml`。本次 .18 配置：左侧张开 -2.8°、闭合 27.2°；右侧 0°/0°，未标定会拒绝动作。其他标定、TCP、动力学和网络参数保留。

位置 0..255 对应张开至闭合角度。速度 0 使用默认速度，默认配置 0 表示使用舵机 0x5D 上限；255 使用有效最大速度，其余值按比例映射。每次 Action 初始化只读 0x5D，配置速度也会被该上限约束；读取失败或上限为零则拒绝动作，不通过手填速度绕过检查。力矩 0 使用 `ds485_gripper_default_pwm`（默认 1000）；1..255 映射到 PWM 1..1000。这是 PWM 上限，不是物理力矩。

状态使用 0x01 和 0x59 列表读取。保护位是 Bit0..6，Bit7 仅表示运动。只下发一次 0x66，不自动返回起点，不重发运动指令。取消、超时或另一侧通信失败会向可能仍在运动的侧发 0x64=2，包括运动命令已发送但 ACK 丢失的情况；锁舵失败会体现在错误结果中。保护故障侧不重新上力。正常到位或检测到夹物后保持目标；几乎没有移动时输出 WARN。应用退出期间停止正在执行的夹爪 Action。

`0x5D` 读取和最大速度/力矩行为仍需硬件验收；右侧需要独立标定。控制链只写运行时 PWM、锁舵和运动寄存器，不保存参数或自动清除保护。

编译并 source 正确安装目录后，操作入口为：

```bash
ros2 run motion_control_pkg ds485_gripper close left
ros2 run motion_control_pkg ds485_gripper open left
ros2 run motion_control_pkg ds485_gripper 128 left 100 50
```

格式：`ds485_gripper <open|close|0..255> <left|right|both> [力矩0..255] [速度0..255] [超时ms]`。省略力矩和速度时使用上述默认值。每条命令会实际驱动夹爪，安装验证不执行这些命令。

离线回归直接使用生产协议和 Action 函数体，替换 SDK 收发为模拟舵机：

```bash
python3 src/motion_control_pkg/test/ds485/run_tests.py --source src/motion_control_pkg --build-dir /tmp/ds485-offline
```

覆盖噪声/分包/坏校验、位置和速度换算、零速度上限拒绝、双侧通道、丢失 ACK、读失败、故障保护、取消、超时、退出和锁舵失败。CTest 入口为 `ds485_offline`。
