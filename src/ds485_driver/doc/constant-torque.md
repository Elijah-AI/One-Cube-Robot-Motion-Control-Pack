# 恒定 PWM 控制

DS485 协议把 `0x6E` 称为电机模式定扭控制。指令给定有符号的开环 PWM，单位为千分之一满幅。实际夹持力随负载、供电和传动变化。需要用 N·m 或 N 表示力矩、夹持力时，还需要独立测量和标定。

工作区基线为 `11923eb8670abd1b76265e3b115fb0bb9594e11f`。x86 的分支目录是 `~/work/ds485_ws_constant_torque`。

## 启动和调用

以下命令在 x86 上执行。使用 Marvin 透传时，运控与夹爪节点共用本机 UDP 4730。

```bash
cd ~/work/ds485_ws_constant_torque
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ds485_driver
source install/local_setup.bash
```

只启用左侧时，直接启动节点并覆盖右侧开关。

```bash
ros2 run ds485_driver ds485_node --ros-args \
  --params-file "$(ros2 pkg prefix --share ds485_driver)/config/ds485_driver.yaml" \
  -p control_strategy:=torque -p right.enabled:=false
```

双侧接线完成后，可按配置启动两侧。左侧为 A、`side=0`，右侧为 B、`side=1`。两条总线分别使用 ID 1。

```bash
ros2 launch ds485_driver ds485_driver.launch.py control:=torque
```

在另一终端加载同一工作区后执行。以下每条命令均单独生效。

```bash
source /opt/ros/humble/setup.bash
source ~/work/ds485_ws_constant_torque/install/local_setup.bash

# 正向输出 100/1000，持续 200 ms，等制动确认后返回
ros2 run ds485_driver servo left torque 100 200

# 反向输出 100/1000，持续 200 ms
ros2 run ds485_driver servo left torque -100 200

# 持续输出。下一条同侧命令更新输出，给 0 停止
ros2 run ds485_driver servo left torque 100
ros2 run ds485_driver servo left torque 0

# 同一接口的 ROS 2 服务调用
ros2 service call /ds485/servo/set_torque ds485_interfaces/srv/ServoSetTorque \
  "{side: 0, pwm: 100, duration_ms: 200}"
```

`pwm` 取 −1000～1000，正负表示电机方向。有效输出受舵机自身配置限制。`duration_ms=0` 表示持续输出。`pwm=0` 始终请求停止。原有位置话题中 `GripperCommand.torque` 仍表示位置控制的 PWM 上限。恒定输出接受负值，并支持独立停止。

## 硬件交接和反馈

1. 节点启动后只采样。收到非零命令后，本侧工作线程读取状态和原模式。确认静止后从制动状态进入 `0x5A=2`，再写一次小端 Int16 到 `0x6E`。
2. `sent` 表示传输接口已受理运动帧。Marvin SDK 受理时会把报文放入发送槽。`acknowledged` 表示驱动收到并校验了成功的写短应答。写应答关闭时，通过状态话题观察实际输出。
3. 正时长从运动帧交给传输层时开始计时。到期、显式停止、活动中失联或收到保护状态时，节点尝试发送零输出和制动。它读取 `0x64=1` 并取得两次零转速、零 PWM、运动标志清除的反馈，再恢复进入时的控制模式。只要制动还无法确认，就会报告 `STOP_UNCONFIRMED`。
4. 正常退出同样会清理本节点正在控制的侧。计时和失联处理在 x86 进程中执行，主机掉电、进程遭强杀或总线中断时，舵机收到停止指令的时间无法保证。

运动期间订阅 `/ds485/state` 可看实际角度、转速、电流和 PWM。`/ds485/gripper/health` 还提供温度、电压、`phase`、`reason` 和 `stop_confirmed`。外部模块也可发布 `/ds485/gripper/stop` 抢占同侧输出。硬件保护由舵机执行，恒定 PWM 控制流程沿用设备现有阈值和运行时上限。

```bash
ros2 topic echo /ds485/state
ros2 topic echo /ds485/gripper/health
ros2 run ds485_driver servo left read 0x5C 2
ros2 run ds485_driver servo left read 0x5D 2
```

空闲时可以用 `servo left speedlimit <度每秒>` 写入并读回 `0x5D`。只有显式调用限幅服务才会修改该值。本次实机测试只读了上限。

本仓库[《第五版协议》](第五版协议.pdf)第 16 页第 3.4.93 节已更正。速度限制是 `0x5D`，单位 °/s。相邻的 `0x5C` 是电流限制，单位 mA。两项均为小端 Uint16。

## 采集报文

需要抓包时，用下面的命令启动左侧节点。在另一终端通过服务调试。日志中的 `tx_hex`、`rx_hex` 是传输层看到的 DS485 请求和应答，`txid` 为本次运行中的事务序号。

```bash
ros2 run ds485_driver ds485_node --ros-args \
  --params-file "$(ros2 pkg prefix --share ds485_driver)/config/ds485_driver.yaml" \
  -p control_strategy:=torque -p right.enabled:=false -p trace_hex:=true \
  --log-level ds485_node:=debug 2>&1 | tee torque-left.log
```

## x86 左侧空载实测

2026-11-19，在 A 臂 COM1、ID 1 上执行了一个非零脉冲。首次准备时，舵机接受 `0x5A=2`，随后对位置保持开关 `0x64=2` 返回 `0x40`。该次非零运动帧尚未发出，节点完成制动并恢复原模式。修正后的流程从制动状态进入电机模式，直接发送 `0x6E`。

| 观察项 | 实测结果 |
| --- | --- |
| 给定 | +100/1000，200 ms |
| 运动帧 | `F9 FF 01 05 03 6E 64 00 24`，短应答 `F9 F5 01 02 80 7C` |
| 运动反馈 | 起点 −2.3°，途中两次回读 PWM +100，速度分别为 20 和 40 °/s |
| 停止帧 | `F9 FF 01 05 03 6E 00 00 88`，短应答 `F9 F5 01 02 00 FC` |
| 停止读回 | +3.6°，PWM 0，转速 0，开关 1，原模式 0 已恢复 |
| 环境与上限 | 停止后 25 ℃、23.886 V。前后均为 PWM 上限 392、电流上限 5000 mA、速度上限 500 °/s、保护使能字 `0x0D` |
| 服务应答 | `success=true`、`sent=true`、`acknowledged=true`、`stop_confirmed=true`、`PULSE_COMPLETE` |

x86 原始抓包在 `~/work/ds485_ws_constant_torque/log/left-torque-4T0JJ2/motor-mode/node-pulse.log`。同目录的 `pulse.txt` 保留参数、服务应答和前后快照。该固件单独读 `0x46` 的一次请求超时，本次实际角度来自 `0x59` 组读的 `0x46` 字段。

双侧伪串口另行验证了反向和持续输出、无写应答、手动停止、抢占、保护触发、断线及正常退出。右侧实机、带载夹持力和持续输出的热行为留待对应工况验证。
