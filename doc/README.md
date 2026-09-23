# ylf_wbc

本工作区全身运控、COM/ZMP 与点云防碰撞用法见 [ylf_wbc 运控命令手册](MOTION_WBC_USAGE.md)。
双臂、身体及头部的 20 关节真实反馈见 [关节反馈说明](MEASURED_JOINT_FEEDBACK.md)。
腿部 J1/J2/J3 任意两关节 MoveJ 自动补偿、全程保持 body 朝向，见 [腿部定姿态 MoveJ](LEG_BODY_ORIENTATION_MOVEJ.md)。
全身 MoveP 可按指令选择只用腰部 J4 和手臂，见 [腰部与手臂 MoveP](WHOLE_BODY_WAIST_ONLY_MOVEP.md)。
笔记本组装件双手抓取、J4 与单臂依次放置，见 [两件抓放任务](LAPTOP_ASSEMBLY_TASK.md)。

2026-09-17：源码统一到 `src/`，视觉按 develop、运控按 HHworkspace 合并。
避障参数改为 `enable_collision_check: 0/1/2`；见 [合并与验证记录](MERGE_20260917.md)。

双臂真机演示的 ROS 2 Humble 主干。核心改动作者：Heng He <elijahhe2000@163.com>。

- `com_interfaces`：统一后的 `MovePose`、`MoveJoints` Action；`ARM_BOTH=2`
- `motion_control_pkg`：双臂运动控制与真机驱动
- `onecube_robot_description`：机器人描述、网格与配置
- `task_manager`：Modbus TCP 任务入口及 Action 适配

本仓库保留功能源码、接口、配置、launch、机器人模型和必要的第三方依赖。测试源码、测试数据、测试构建目标及备份包已移出工作区；构建产物和机器人归档不提交到 Git。克隆后在仓库根目录生成标准 `build/`、`install/`、`log/`。

2026-09-21：同步机器人 `172.16.21.80` 的点云持久订阅、后台接收和缓存实现，保留 WBC、反馈、运控及视觉任务接口。

## 编译

```bash
cd /path/to/ylf_wbc
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
source install/setup.bash
```

不要叠加旧工作区、单独的 task_manager 安装空间或临时构建目录。

## 启动真机

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /path/to/ylf_wbc/install/setup.bash
ros2 launch motion_control_pkg bringup.launch.py active_arm_configuration:=dual_arm
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /path/to/ylf_wbc/install/setup.bash
ros2 launch task_manager task_manager.launch.py \
  config_path:=/path/to/ylf_wbc/src/task_manager/config/task_config.yaml
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
