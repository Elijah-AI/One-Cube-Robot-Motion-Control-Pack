# 2026-11-04 机器人运控源码同步

本次来源为 `it@192.168.10.18:/home/it/ylf_wbc`。按用户确认，以机器人版本为准；三个核心包及配套 x86 DriverSDK 的 454 个文件逐字节核对一致，符号链接同时核对链接目标。文件清单见 [ROBOT_SOURCE_20261104.sha256](ROBOT_SOURCE_20261104.sha256)。

同步范围：

- `src/com_interfaces`：28 个文件。
- `src/motion_control_pkg`：350 个文件，包含配套离线回归源码。
- `src/onecube_robot_description`：66 个文件。
- `vendor/driver_sdk`：10 个文件或库链接，包含匹配的头文件、EtherCAT 库及第三方依赖。

首次核对发现 41 个现有文件不同、11 个机器人文件本地缺失。同步前已在仓库外备份本地差异；本地尚未部署的独立 LEG TCP 水平补偿及旧 `error` 入口移出当前运控目录。本次上传沿用 GitHub `main` 的历史与 LICENSE，另同步 `task_manager` 的 WBC 水平约束字段解析与转发，并去除本地 LEG 扩展校验引用，使其使用机器人版本的接口校验；其他业务包沿用原仓库版本。

## 主要行为

- `arm_error` 控制双臂，`body_error` 控制腿、腰、头；两组每次进程启动均默认 OFF，空闲时通过原子服务配置限制。
- 腰部 `body_joint` 的 URDF 速度上限为 `1.31 rad/s`；头部 yaw/pitch 均为 `0.61 rad/s`。精到位慢段与启动回零速度分别配置。
- 头部启动回零专用 pitch 下界为 `-0.7 rad`，上界仍按 URDF，回零指令速度上限为 `0.1 rad/s`。
- SIGINT/SIGTERM 采用有序退出：拒绝新命令、等待实测静止、失能并再次确认静止，然后释放驱动。流程见 [ORDERLY_SHUTDOWN.txt](../src/motion_control_pkg/doc/ORDERLY_SHUTDOWN.txt)。
- TCP 水平约束仅支持 WBC MovePose（`arm_id=5/6/7`）；所有 MoveJoints 开启该约束均被拒绝。
- 规划线程启动参数仍支持 `1/2/4`，默认 `2`，详见[完整手册第 2.5 节](MOTION_COMMANDS_AND_PARAMETERS.md#25-规划线程数与四线程启动)。

源码及 SDK 保留机器人原始字节。手册另外校正旧回零默认值、旧误差开关名称及未部署的 LEG TCP 补偿描述，并保留已有四线程规划说明。

## 构建与核验

在 ROS 2 Humble x86 环境、已有项目依赖可用的条件下，配套 SDK 的构建选项为：

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$PWD/vendor/driver_sdk/igh_install/lib:${LD_LIBRARY_PATH:-}"
colcon build --executor sequential --packages-select com_interfaces onecube_robot_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon build --executor sequential --packages-select motion_control_pkg \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_TESTING=ON -DONECUBE_USE_REAL_DRIVER_SDK=ON \
    -DONECUBE_USE_REAL_MARVIN_SDK=ON \
    -DONECUBE_DRIVER_SDK_ROOT="$PWD/vendor/driver_sdk"
ctest --test-dir build/motion_control_pkg --output-on-failure -j1
sha256sum -c doc/ROBOT_SOURCE_20261104.sha256
```

资源受限机器还应设置 `CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 NINJAFLAGS=-j1`，并使用 `cmake --build build/<包名> --parallel 1` 控制包内编译并行数。该 SDK 快照为 x86；ARM64 构建须选用对应架构的匹配 SDK。

本次仅同步本地源码、离线构建与发布 GitHub，未修改机器人文件、重启控制器或发送运动命令。构建、接口响应和离线测试不能替代实机运动、停止及抱闸验收。

## 本次本地验证结果

- ROS 2 Humble x86 隔离构建：接口包、机器人模型、真实 SDK 运控节点及任务节点编译通过。
- 运控 20 个不同 CTest 用例全部通过。首次运行 19 项通过；`shutdown_launch` 因隔离目录缺少安装包索引失败，补齐隔离模型及运控安装后单项重跑通过。
- 20 项既有任务 YAML 解析场景通过；默认关闭行为保留，机器人尚未部署的 LEG TCP 水平约束被拒绝。
- 运控与任务节点的 `ldd -r` 均无缺库及未解析符号；发布文档链接、Python/XML 解析及 27 段 Bash 示例检查通过。
- `bringup.launch.py --show-args` 确认规划线程支持 1/2/4，默认 2。仅显示参数，未启动运控节点。
- 发布前再次读取机器人哈希，454 个核心文件与本地及 Git 暂存版本保持一致。
