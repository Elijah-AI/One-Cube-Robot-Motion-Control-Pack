# 运控直接调用 DS485 C++ 接口

共享 SDK 的位置控制沿用现场 C++ 接口，恒定 PWM 采用 `/home/it/work/ds485_ws` 的 `93f0b749`。运控在 `/home/it/ylf_wbc` 构建同工作区的 `src/ds485_driver`。
集成方式是同一进程内的普通 C++ 函数调用。运控与夹爪驱动之间没有 ROS 命令转发或额外进程。

## 代码职责

| 位置 | 职责 |
|---|---|
| `ylf_wbc/src/ds485_driver/src/rs485` | Marvin 末端 A/B 臂 COM1/COM2 的收发 |
| `src/servo` | 德晟帧、寄存器和舵机指令 |
| `src/gripper/ds485_gripper.cpp` | 开度换算、到位和阻挡判断 |
| `src/gripper/gripper_controller.cpp` | 每侧一个执行线程，接纳、取消、超时、状态与结果 |
| `src/gripper/gripper_config.cpp` | 独立 YAML 读取和校验，不访问硬件 |
| 运控 `src/common/gripper/ds485_ros_bridge.cpp` | 对外 Action/topic 转为本进程 C++ 调用，结果回给调用者 |

驱动目录是底层实现的唯一来源，运控不再保存 `vendor/ds485_driver` 副本。
构建产物为 `libds485_control.a`，导出目标 `ds485_driver::ds485_control`。
该库不链接另一份 MarvinSDK：最终可执行程序提供自己的 SDK。
独立节点和运控各自管理所在进程的 SDK。
修改驱动后必须重新编译驱动库，再重新链接运控；仅改独立 YAML 时重启运控即可生效。

## SDK 生命周期

1. 运控按原流程检查并建立 SDK 连接，只调用一次 `OnLinkTo()`。
2. 运控调用 `GripperController::start()`，驱动内部执行 `MarvinLink::Attach()`。
3. 夹爪只使用这份连接的透传接口；两侧各自串行访问所属总线。
4. 退出时先 `shutdown()`，取消尚未完成的命令、等待夹爪线程结束。
5. 驱动解除 Attach，不调用 `OnRelease()`；最后由运控释放 SDK。

集成运行时只启动运控，不启动 `ds485_node`。
独立的 `ds485_node` 可继续用于单独调试，其启动方式仍为 `ros2 launch ds485_driver ds485_driver.launch.py`；
它仍会自行建立 SDK 连接，因此使用 Marvin 透传时不能与运控并行运行。

## 运控内的恒定 PWM

`ds485_control_strategy:=torque` 使用 `ds485_driver::EmbeddedTorque`。它在宿主进程中接入已有 MarvinSDK，让两侧总线各自处理请求。非零 PWM 请求先经过运控接纳检查，零 PWM 和制动请求可以打断原指令。位置与恒定 PWM 的启动参数和调用示例见[运控指令与参数](../../../doc/MOTION_COMMANDS_AND_PARAMETERS.md#511-恒定-pwm)。

## 位置控制的公开 API

包含 `ds485_driver/gripper_config.h` 和 `ds485_driver/gripper_controller.h`，使用 C++17。
接口只使用普通结构体、标准容器、互斥量和线程，没有 ROS 类型。

| 接口 | 行为 |
|---|---|
| `LoadGripperConfig(path, config, error)` | 读文件并校验；失败返回 false 且不改写 config |
| `start()` | Attach 后启动线程；要求宿主已连接 SDK，不主动移动 |
| `submit(commands, handles, error)` | 同时检查请求侧的资源，成功返回对应句柄；不等待动作完成 |
| `outcome(handle)` | 直接返回本条命令的完成标志、结果、错误、末次状态 |
| `state(side)` | 直接返回缓存状态，含 online/busy/position/servo/error |
| `cancel(handles)` | 只取消这些句柄对应的命令，不影响之后的命令 |
| `stop(side)` | 0 左、1 右、2 两侧；取消命令并请求锁舵 |
| `busy(side)` | 资源是否占用或存在未确认停止 |
| `shutdown()` | 取消并 join，完成后宿主才能释放 SDK；可重复调用 |

直接调用示意（仅供阅读，运行会驱动夹爪；宿主已持有 SDK 连接）：

```cpp
#include "ds485_driver/gripper_config.h"
#include <chrono>
#include <thread>

using ds485_driver::DriverConfig;
using ds485_driver::GripperController;

DriverConfig settings;
std::string error;
if (!ds485_driver::LoadGripperConfig(config_path, settings, error)) {
    // 将 error 返回上层并结束初始化。
    return;
}

GripperController driver(settings.sides, {}, {}, {}, settings.poll_ms,
                         settings.state_rate_hz);
driver.start();  // 宿主已连接 SDK，这里只 Attach。

GripperController::Command command;
command.side = 0;
command.position = 255;
command.torque = 100;
command.speed = 50;
command.timeout_ms = settings.default_timeout_ms;
command.action = true;  // 独占所请求侧，不被后续 topic 类命令替换。

std::vector<GripperController::Handle> handles;
if (driver.submit({command}, handles, error)) {
    GripperController::Outcome result;
    do {
        result = driver.outcome(handles[0]);
        if (!result.done) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (!result.done);
    // result.success()、result.result、result.message、result.state 直接返回给运控。
}
driver.shutdown();  // 宿主在这之后释放 SDK。
```

运控适配器额外在等待期间发布 Action feedback、处理取消；无需用户自行编写上述循环。
Result 数值：1 到位、2 阻挡、3 故障、4 超时、5 失败、6 取消、7 被替换。
只有 1/2 的 `success()` 为 true。`online=false` 时状态中的旧数值不能视为新测量。
`started` 表示进入下发阶段，不表示舵机已到位；完成应以 `done` 和 `result` 为准。

## 独立配置

正式文件：`/opt/onecube/motion_control/config/ds485_gripper.yaml`。
运控主配置仅保留：

```yaml
gripper_type: ds485
ds485_gripper_config: /opt/onecube/motion_control/config/ds485_gripper.yaml
```

文件沿用独立节点的 `ds485_node: ros__parameters:` 结构，下面是分组含义：

| 字段 | 单位 / 约束 |
|---|---|
| `poll_ms` | 工作线程等待间隔，1..1000 ms |
| `state_rate_hz` | 空闲状态读取频率，0 关闭或 0.1..100 Hz |
| `default_timeout_ms` | topic 未指定超时时使用；1..4294967295 ms |
| `left/right.enabled` | 是否启用该侧；运控的工具类型也必须为 gripper |
| `port_type`, `arm` | 集成模式要求 marvin；左 A、右 B |
| `channel`, `servo_id` | COM1=2 / COM2=3；舵机 ID 0..250 |
| `gap_ms`, `timeout_ms` | 每次发送间隔 0..1000 ms；单次应答等待 10..5000 ms |
| `open_deg`, `close_deg` | 舵机角度，度；启用侧必须不同且符合协议范围 |
| `default_pwm` | torque=0 时的 PWM 上限，1..1000，不是物理力矩 |
| `default_speed`, `max_speed` | 度/秒；0 使用舵机速度上限，最大 65535 |
| `tolerance_deg`, `still_band_deg`, `blocked_samples` | 到位容差、不动角度带、连续样本数 |

当前机器人标定：左开/闭 0.2°/98.1°，右开/闭 57.9°/167.6°。
位置 0 全开、255 全闭；torque/speed 的 1..255 为比例值，0 为上述默认值。
读不到舵机 0x5D 速度上限或上限为零时拒绝运动，配置值不能绕过该检查。
`marvin_ip` 只供独立节点连接时使用；集成时地址由运控管理。
`serial_device/serial_baud/allow_raw_write` 是独立调试模式字段，集成模式不启用串口或原始写服务。

左右侧各自读取 `protection`。本机左侧为 5–30 V、60 °C（回差 5 °C）、6000 mA 持续 200 ms、堵转 PWM 600 持续 3000 ms。右侧各阈值为 `-1`，保留舵机现值。两侧 `auto_recover` 均启用。启动或重新连通时按差异同步并读回保护设置，运行中的命令不会自动重试。新出现的保护位会记录温度、电压、电流和 PWM。

## 对外 ROS 接口与日志

运控保留 `com_interfaces/action/ControlGripper` 的 `/Control/control_gripper`，字段不变。
Task ID 非空，数组至少两项，`block_until_complete=true`，`com_port` 必须匹配配置。
Action 的结果从驱动 C++ Outcome 直接转换，不依赖 `/ds485/gripper/result` 的接收。
手动 topic 使用 `ds485_interfaces`，保留 command/stop/result/state 四个原名称。
同侧 Action 与 topic 互斥；topic 可替换先前 topic，先请求旧目标锁舵，再执行新目标。
Action 双侧请求中任一侧失败，会取消另一侧未完成的命令。

保留配置错误、命令失败、离线及舵机保护的警告/错误；重复状态不反复刷日志。
取消、超时、应答丢失及退出会尝试锁舵；保护位存在时不会重新使能舵机。
不能确认锁舵时拒绝后续动作，现场检查后重新启动。可恢复保护的复核由下一条新运动指令触发。
软件的锁舵应答检查不能替代现场的实际停止与夹持验收。

## 编译与检查

在 `ylf_wbc` 内依次串行构建 `ds485_interfaces`、`ds485_driver` 和 `motion_control_pkg`。
`find_package(ds485_driver REQUIRED)` 和链接 `ds485_driver::ds485_control` 是运控的唯一底层依赖。
正式构建脚本在本次交付目录中，明确使用 `cmake --build ... --parallel 1`。
独立库的 CTest：`ds485_offline_test`、`ds485_controller`、`ds485_config`；
运控的 `ds485_ros_bridge` 使用真实 ROS 通信和 Fake 总线，localhost/domain 187，不连接硬件。
构建、安装和真机启用分别记录；本次修改不自动启动运控或发送动作。
