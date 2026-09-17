# ros2_ylf_demo

2026-09-17：源码统一到 `src/`，视觉按 develop、运控按 HHworkspace 合并。
避障参数改为 `enable_collision_check: 0/1/2`；见 [合并与验证记录](MERGE_20260917.md)。

双臂真机演示的 ROS 2 Humble 主干。核心改动作者：Heng He <elijahhe2000@163.com>。

- `com_interfaces`：统一后的 `MovePose`、`MoveJoints` Action；`ARM_BOTH=2`
- `motion_control_pkg`：双臂运动控制与真机驱动
- `onecube_robot_description`：机器人描述、网格与配置
- `task_manager`：Modbus TCP 任务入口及 Action 适配

源码单元测试保留用于回归验证；旧工作区的 `build*`、`install*`、`log*`、临时副本和测试部署环境均未提交。克隆后只在仓库根目录生成一套标准 `build/`、`install/`、`log/`。

## 编译与测试

```bash
cd /path/to/ylfrobot
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

不要叠加旧工作区、单独的 task_manager 安装空间或临时构建目录。

## 启动真机

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /path/to/ylfrobot/install/setup.bash
ros2 launch motion_control_pkg bringup.launch.py active_arm_configuration:=dual_arm
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /path/to/ylfrobot/install/setup.bash
ros2 launch task_manager task_manager.launch.py \
  config_path:=/path/to/ylfrobot/task_manager/config/task_config.yaml
```

## 发送任务

当前本机任务服务地址为 `192.168.1.123:12000`，站号为 `1`。寄存器文档编号为 40001/50001，`mbpoll` 使用零基偏移，因此对应 `-r 1`/`-r 10001`。

```bash
# 只读检查触发寄存器（应为 0）
mbpoll 192.168.1.123 -p 12000 -a 1 -t 4 -r 1 -c 1 -1

# 只读检查状态寄存器：0=空闲，1=执行中，2=成功，3=失败
mbpoll 192.168.1.123 -p 12000 -a 1 -t 4 -r 10001 -c 1 -1

# 写入一次触发值 1；-o 2 设置 2 秒超时
mbpoll 192.168.1.123 -p 12000 -a 1 -t 4 -r 1 -1 1 -o 2

# 完成后复查状态
mbpoll 192.168.1.123 -p 12000 -a 1 -t 4 -r 10001 -c 1 -1
```

当前 `move_box` 是两个并行分支：左、右臂分别发送单臂 `MoveJoints`，目标均为 Joint1=`0.8 rad`、其余关节=`0`，速度比例 `0.2`，碰撞检查关闭。真机运行前必须确认双臂各自在人工划定区域内，现场人员可随时急停。
