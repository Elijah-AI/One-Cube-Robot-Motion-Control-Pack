# berxel_camera_driver

基于 **Berxel Hawk (iHawk) 3D 深度相机 Linux SDK** 的 ROS2 相机驱动节点，
功能与现有 Orbbec 版 [`camera_driver`](../camera_driver/README.md) **完全一致**，
**唯一区别是底层图像原始数据来源**：本包使用 Berxel Hawk SDK（`libBerxelHawk.so`）替代 Orbbec SDK。

- 节点名 / 可执行文件：`camera_driver_node`
- 部署目标：RK3576（aarch64 / arm64）。SDK 动态库按架构随包内嵌（`SDK/x86` 与 `SDK/arm`），
  CMake 在配置期自动选择：`aarch64 → SDK/arm`，`x86_64 → SDK/x86`。
- **WSL / x86_64 host 上可原生完整编译链接通过（无需交叉编译）**；板端在 RK3576 上直接原生构建。
- Berxel SDK 已**随包内嵌**（见 [`SDK/`](SDK/)），构建与运行无需再单独下载 / 配置 SDK 路径。

---

## 1. 目录结构

```
berxel_camera_driver/
├── CMakeLists.txt                      # ament_cmake 构建；默认使用包内本地 SDK
├── package.xml
├── README.md
├── SDK/                                # 随包内嵌的 Berxel Hawk SDK（自包含，仿 camera_driver/SDK）
│   ├── Include/                        # 共享 C++ 头文件（5 个 BerxelHawk*.h，两架构通用）
│   │   ├── BerxelHawkContext.h
│   │   ├── BerxelHawkDefines.h
│   │   ├── BerxelHawkDevice.h
│   │   ├── BerxelHawkFrame.h
│   │   └── BerxelHawkPlatform.h
│   ├── x86/                            # x86_64 动态库 + 标定参数（WSL / x86_64 host 原生构建）
│   │   ├── libBerxelHawk.so            # 主库（直接链接，x86-64）
│   │   ├── libBerxelInterface.so       # 接口层（被 Hawk dlopen）
│   │   ├── libBerxelCommonDriver.so    # 通用驱动（被 Interface dlopen）
│   │   ├── libBerxelLogDriver.so       # 日志驱动（被 dlopen）
│   │   ├── libBerxelNetDriver.so       # 网口设备驱动（被 dlopen）
│   │   ├── libBerxelUvcDriver.so       # USB/UVC 设备驱动（被 dlopen）
│   │   └── params*.bin                 # 相机标定参数（params.bin / 100E / 100Q / 137_0 / 137_1）
│   ├── arm/                            # aarch64 动态库 + 标定参数（RK3576 板端原生构建）
│       ├── libBerxelHawk.so            # 主库（直接链接，aarch64）
│       ├── libBerxelInterface.so       # 接口层（被 Hawk dlopen）
│       ├── libBerxelCommonDriver.so    # 通用驱动（被 Interface dlopen）
│       ├── libBerxelLogDriver.so       # 日志驱动（被 dlopen）
│       ├── libBerxelNetDriver.so       # 网口设备驱动（被 dlopen）
│       ├── libBerxelUvcDriver.so       # USB/UVC 设备驱动（被 dlopen）
│       └── params*.bin                 # 相机标定参数（params.bin / 100E / 100Q / 137_0 / 137_1）
│   ├── berxelCamera.ini                # SDK 热插拔扫描配置（arch 无关，随 .so 装到 <prefix>/lib）
│   └── berxelLog.ini                   # SDK 日志配置（arch 无关，随 .so 装到 <prefix>/lib）
├── config/
│   └── berxel_camera_driver.yaml       # 参数模板文件（供 launch config_file:= 或 ros2 run --params-file 使用）
├── launch/
│   └── berxel_camera_driver.launch.py  # 启动文件（与 camera_driver.launch.py 参数一致）
├── include/berxel_camera_driver/
│   └── camera_driver_node.hpp          # CameraDriverNode 类声明
├── src/
│   └── camera_driver_node.cpp          # 节点实现
└── udev/
    ├── berxel-usb.rules                # Berxel USB 权限 udev 规则（首次运行每台机器安装一次）
    └── install_berxel_udev.sh          # 一键安装 udev 规则（需 root，安装后须拔插相机）
```

> 为什么 SDK 要整体内嵌且安装到 `<prefix>/lib`？
> - `libBerxelHawk.so` 直接 `NEEDED libBerxelInterface.so`；
> - 其余驱动库（Interface/CommonDriver/LogDriver/NetDriver/UvcDriver）在运行期**按名字 `dlopen`**；
> - `libBerxelUvcDriver/NetDriver` 运行期读取 `params*.bin` 标定文件。
>
> 因此 6 个 `.so`、5 个 `params*.bin` 以及 2 个 arch 无关的 `berxelCamera.ini`/`berxelLog.ini`
> 必须**全部**放在 `ament` 能通过 `LD_LIBRARY_PATH` 找到的目录（即 `<prefix>/lib`，
> `source install/setup.bash` 后自动追加）。这正是官方 `RosSDKNode/berxel_camera_ros2`
> wrapper 与 `camera_driver/SDK` 的安装方式；`libBerxelUvcDriver.so` 会在**自身所在目录**
> （即 `<prefix>/lib`）查找 `berxelCamera.ini`，故 ini 必须与 `.so` 同目录。
> SDK 自带动态库内置了无效 RPATH（编译机路径），**不能依赖**其 RPATH 解析。
> 非 root 使用相机需先安装 USB udev 规则（见 §6.1，每台机器一次）。

---

## 2. 功能特性（与 camera_driver 一致，仅数据源不同）

- 枚举并打开 Berxel 设备；支持**多相机**：
  - 单相机：`serial_number=""` 时自动打开第一台；
  - 多相机：`camera_id_sn_map`（下标 = camera_id，值 = SN）逐台按 SN 打开。
- 每台相机以 **MIX_HD（color + depth）** 启动，分辨率/帧率由参数指定（默认 1280x800@5fps，可运行期覆盖）。
- **帧模式自动校正**：启动时读取 SDK 支持列表（`getSupportFrameModes`），若请求的分辨率/帧率不被设备
  支持（iHawk100RS 的 color 仅支持 1280x800@5..45，旧的 1280x720@3fps 请求会被拒绝导致无帧），自动挑选
  像素面积最近、帧率最接近的可支持模式，并打印 `Supported Color/Depth modes` 与 `auto-selected nearest
  supported` 日志；`setFrameMode` 被拒绝时显式报错返回，不再静默无帧。
- **深度到彩色对齐（D2C）**：Berxel SDK 仅支持硬件注册 `setRegistrationEnable(true)`，
  故 `align_mode=HW/SW` 均映射为硬件对齐，`OFF` 关闭；点云与 color 图逐像素对齐（位于彩色光学坐标系）。
- **主机时间同步**：`enable_host_time_sync=true` 时周期性调用 `setSystemClock()`，
  将相机内部时钟同步到主机系统时间，使图像时间戳与机械臂 `/Control/*` 数据对齐。
- **内部帧融合**：同一相机匹配 color/depth 帧对后，把机械臂关节/末端位姿与图像按主机时间戳融合，
  经 `/fusion/synced_robot_vision` 发布（含机械臂运控数据的单话题同步视觉）。
- 头部位姿订阅 `/Control/HeadPose`，对点云做**机械臂基坐标系变换**（眼在手标定）。
- **心跳 / 状态**：1Hz `/SafeNode/Heartbeat`，状态变化时发布 `/SafeNode/status`。
- 相机故障自动**复位/重连**，并对外暴露动作服务供上层自检。
- 跨进程设备锁（`bypass_lock=false` 时基于共享内存加锁，防止多进程争用设备）。

---

## 3. ROS2 接口

### 3.1 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/fusion/synced_robot_vision` | `com_interfaces/msg/SyncedRobotVision` | 融合后的 RGB 图 + RGB 点云 + CameraInfo + 机械臂运控数据（关节、末端位姿）；多相机用 `camera_id` 区分 |
| `/vision/rgb_raw` | `sensor_msgs/msg/Image` | 主相机（camera_id=0）RGB 原图；**仅当存在订阅者时**才发布 |
| `/SafeNode/Heartbeat` | `com_interfaces/msg/HeartBeat` | 心跳，1Hz |
| `/SafeNode/status` | `com_interfaces/msg/NodeStatus` | 节点状态机状态 + 错误标志，状态变化时发布 |

`SyncedRobotVision` 字段说明（`camera_id`：0=头部，1=腰部，2+ 预留）：

```
std_msgs/Header header                      # 头部时间戳
uint8 camera_id                             # 相机 ID
sensor_msgs/Image camera_rgbd_color         # RGB 彩图
sensor_msgs/PointCloud2 camera_rgbd_pointcloud  # RGB 点云（米制，位于彩色光学坐标系）
sensor_msgs/CameraInfo camera_info          # 相机内参/畸变/投影矩阵
string target_label; float32 seg_confidence; bool data_valid   # 预留
string[] joint_names                        # 关节名称
float64[] joint_positions / joint_velocities / joint_efforts   # 关节数据
geometry_msgs/Pose[] end_effector_pose      # 末端在基坐标系下的位姿
```

### 3.2 订阅话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/Control/MotorInfo` | `com_interfaces/msg/MotorStatus` | 机械臂关节/末端数据，用于帧融合 |
| `/Control/HeadPose` | `com_interfaces/msg/MotorStatus` | 头部末端位姿（取 `end_effector_pose[0]`），用于点云基坐标变换 |

### 3.3 Action 服务

| Action | 类型 | 说明 |
|---|---|---|
| `/vision/Check_self` | `com_interfaces/action/HardwareCheck` | 硬件自检：枚举/打开设备、读内参与序列号、校验图像帧，逐相机反馈进度 |
| `/vision/Reset` | `com_interfaces/action/HardwareReset` | 硬件复位：关闭全部设备并重新枚举/打开，恢复帧流 |

---

## 4. 参数

launch 与 `config/berxel_camera_driver.yaml` 提供同一套参数（下表为 launch/yaml 默认值）。
节点内建默认分辨率为 1280x720@30fps（与 `camera_driver` 的 hpp 内建默认完全一致，
仅在 `ros2 run` 不带参数文件时生效）；通过 `ros2 launch` 启动时被 launch/config 默认
覆盖为 1280x800@5fps。由于 iHawk100RS 的 color 不支持 720 高度，无论哪种入口，节点都会
按 SDK 支持列表自动校正为设备支持的最近模式（见上文的“帧模式自动校正”）。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `color_width` / `color_height` / `color_fps` | `1280` / `800` / `5` | 彩色流分辨率与帧率（iHawk100RS color 仅支持 1280x800@5..45） |
| `depth_width` / `depth_height` / `depth_fps` | `1280` / `800` / `5` | 深度（对齐目标）分辨率与帧率；请求不被支持时节点自动校正为最近可用值 |
| `serial_number` | `""` | 为空自动选择第一台设备 |
| `camera_id_sn_map` | `[]` | 多相机 SN→camera_id 映射（数组下标=camera_id）；非空时逐台打开 |
| `frame_id` | `camera_link` | 融合消息 header.frame_id |
| `color_frame_id` | `camera_color_optical_frame` | 彩色光学坐标系 |
| `depth_frame_id` | `camera_depth_optical_frame` | 深度光学坐标系 |
| `bypass_lock` | `false` | 是否跳过跨进程设备锁 |
| `enable_align` | `true` | 兼容旧参数；`align_mode` 优先生效 |
| `align_mode` | `HW` | `OFF` / `HW` / `SW`（Berxel 只有硬件注册，HW/SW 均映射硬件对齐） |
| `align_target_stream` | `COLOR` | 对齐目标流（Berxel 仅支持 D2C→COLOR；请求 DEPTH 会告警并回退） |
| `pointcloud_format` | `rgb_point` | `rgb_point`（xyz float32+rgb uint32）\| `point`（仅坐标） |
| `pc_keep_invalid_points` | `true` | `true` 保留全部像素（无效深度=NaN，width=图像宽）；`false` 仅有效点 |
| `pointcloud_base_frame_id` | `base_link` | 点云经头部位姿变换后的目标坐标系 |
| `enable_host_time_sync` | `true` | 相机内部时钟同步到主机时间 |
| `time_sync_period` | `60.0` | 时间同步周期（秒） |

---

## 5. 构建

### 5.1 SDK 发现与架构选择（CMakeLists.txt）

Berxel SDK 已随包内嵌于 `SDK/`（`Include/` + `x86/` + `arm/`），CMake 按以下优先级定位：

1. `-DBERXEL_SDK_DIR=<SDK根目录>`（显式指定，可指向包外另一 SDK 版本；
   外部 SDK 根目录若为官方平铺 `libs/` 布局也会被自动识别）；
2. 环境变量 `BERXEL_SDK_LIBRARY`；
3. 默认使用包内 `SDK/`（`${CMAKE_CURRENT_SOURCE_DIR}/SDK`，不存在则报错）。

**架构选择**：配置期自动判定目标 CPU —— 优先取 `CMAKE_SYSTEM_PROCESSOR`
（原生构建由 CMake 自动预填；交叉构建由 toolchain 文件设置），兜底用
`uname -m` + `getconf LONG_BIT`（同 camera_driver 的写法）：
`aarch64/arm64 → SDK/arm`，`x86_64/amd64 → SDK/x86`。配置期会打印
`Berxel SDK arch : x86|arm` 便于核对；仅所选架构的库会被安装，避免同机混装两套 `.so`。

### 5.2 在 RK3576（aarch64）上原生构建（推荐）

SDK 已内嵌，**无需单独下载 / 指定 SDK 路径**：

```bash
# 1) 加载 ROS2 Humble
source /opt/ros/humble/setup.bash

# 2) 进入工作空间构建本包
cd ~/ros2_ws
colcon build --packages-select berxel_camera_driver

# 3) 加载 install 环境（ament 会把本包 lib/ 追加到 LD_LIBRARY_PATH）
source install/setup.bash
```

安装布局：

- 可执行文件 `camera_driver_node` → `install/berxel_camera_driver/lib/berxel_camera_driver/`
  （设 `INSTALL_RPATH="$ORIGIN/.."` 作为兜底）；
- SDK `*.so` 与 `params*.bin` → `install/berxel_camera_driver/lib/`
  （`source install/setup.bash` 后经 `LD_LIBRARY_PATH` 可被 `dlopen`/读取）；
- SDK 头文件 → `include/berxel_camera_driver/`；
- launch / config → `share/berxel_camera_driver/`。

### 5.3 在 x86_64 host / WSL 上原生构建（开发验证）

SDK 已内嵌 x86_64 库（`SDK/x86/`），WSL（x86_64）上**直接原生完整编译链接**即可，
无需交叉编译、无需 aarch64 sysroot：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select berxel_camera_driver
```

CMake 检测到 `x86_64` 即选用 `SDK/x86/` 链接（配置期打印 `Berxel SDK arch : x86`）。
产物自检：

```bash
file install/berxel_camera_driver/lib/berxel_camera_driver/camera_driver_node
# ELF 64-bit LSB pie executable, x86-64 ...（NEEDED libBerxelHawk.so，RUNPATH=$ORIGIN/..）
readelf -d install/berxel_camera_driver/lib/berxel_camera_driver/camera_driver_node \
    | grep -Ei 'NEEDED|RUNPATH'
file install/berxel_camera_driver/lib/libBerxelHawk.so      # ELF ... x86-64
```

> 说明：本包当前采用“目标平台各自原生构建”（板端 aarch64 用 §5.2，WSL/x86 用本节），
> `arm/` 与 `x86/` 两套库均随包内嵌，无需交叉编译。若确需在 x86 上交叉编译到 aarch64，
> 仍可参考工作空间根 [`scripts/build_arm64.sh`](../scripts/build_arm64.sh) 与
> [`.cross/aarch64_toolchain.cmake`](../.cross/aarch64_toolchain.cmake)
> （toolchain 设置 `CMAKE_SYSTEM_PROCESSOR=aarch64` 后 CMake 会自动改选 `SDK/arm`）。

---

## 6. 运行

> **首次运行前置（每台机器一次）**：Berxel 相机经 libusb 访问，USB 设备节点默认属 root，
> 普通用户打不开会令 SDK **枚举不到相机**（节点报 `Failed to query device list (ret=-1)`，
> SDK 控制台日志 `Not find device!`）。先安装随包携带的 udev 规则，再**拔插**相机：
>
> ```bash
> PREFIX=$(ros2 pkg prefix berxel_camera_driver)
> sudo bash $PREFIX/share/berxel_camera_driver/udev/install_berxel_udev.sh
> # 脚本将 99-berxel-usb.rules 拷到 /etc/udev/rules.d 并重载；完成后拔下再插上相机。
> lsusb     # 应能看到 0603 / 3558 / 04b4 等 Berxel 相关 VID
> ```
>
> 仍枚举不到时按序排查：`lsusb` 是否出现相机 VID/PID → 该 PID 是否在
> `udev/berxel-usb.rules` 内（不在则需补规则）→ 换 USB 口 / 确认供电。
>
> **流启动后设备掉线（帧数持续为 0）**：若两条流都 `berxelUVCStartStream Success`、状态进入 IDLE，
> 随后约 1 秒内 `Hotplug device ... LEFT` 且 3558 以**新的 DEV 号**重新 `ARRIVE`（0603 却不再回来），
> 之后 `FPS Monitor` 报 “No frames received in the last 5 seconds!” —— 这是 **USB 物理掉线/重枚举**，
> 节点仍握着掉线前的旧设备实例、不会自动重开新设备。按序排查：
> ① 在掉线窗口执行 `sudo dmesg | tail -n 80` 抓根因（`USB disconnect`、`over-current change`、
> `device descriptor read/64, error -71` 等），把输出贴出来；
> ② 相机**直连主板 USB 口**，或接**带独立供电的 USB HUB** —— 无源 hub（本机可见 Genesys 0x1a40）
> 供电不足会让双通道相机首帧即复位掉线（SDK 自检日志也会提示检查 hub 供电）；
> ③ 换线 / 重新插紧；④ 重装 udev 规则（本次已含 `ATTR{power/control}="on"` 禁 USB 自动挂起）后
> 拔插再跑。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 默认参数（1280x800 @5fps；设备不支持时会自动校正为最近可用模式）
ros2 launch berxel_camera_driver berxel_camera_driver.launch.py

# 用与 camera_driver launch 相同的参数覆盖
ros2 launch berxel_camera_driver berxel_camera_driver.launch.py \
    color_width:=640 color_height:=480 color_fps:=30 \
    depth_width:=640 depth_height:=480 depth_fps:=30 align_mode:=OFF

# 或整份加载参数文件
ros2 launch berxel_camera_driver berxel_camera_driver.launch.py \
    config_file:=$(ros2 pkg prefix berxel_camera_driver)/share/berxel_camera_driver/config/berxel_camera_driver.yaml

# 不经过 launch，直接运行节点（用节点内建默认参数）
ros2 run berxel_camera_driver camera_driver_node
```

> `camera_id_sn_map` 默认空（= 不指定 SN，按枚举顺序自动打开第一台设备）。由于
> launch_ros 的 `evaluate_parameters` 不允许空序列（空数组 `'[]'` 会抛
> “Expected a non-empty sequence”），本 launch 在数组为空时**不再显式下发**该参数，
> 让节点使用内建默认 `std::vector<std::string>{}` —— 语义与空数组完全一致。
> 需要按 SN 逐台打开时再显式传入 JSON 数组字符串：
>
> ```bash
> ros2 launch berxel_camera_driver berxel_camera_driver.launch.py \
>     camera_id_sn_map:='["SN1000000","SN2000000"]'
> ```

检查：

```bash
ros2 node list                                    # 应看到 /camera_driver_node
ros2 topic list | grep -E 'fusion|vision|SafeNode'
ros2 action list                                  # /vision/Check_self /vision/Reset
ros2 topic echo /fusion/synced_robot_vision --once
ros2 topic hz /fusion/synced_robot_vision
```

rviz2 可视化（Add → By topic）：

- `/fusion/synced_robot_vision` 的 `camera_rgbd_color` 字段（Image）
- `/fusion/synced_robot_vision` 的 `camera_rgbd_pointcloud` 字段（PointCloud2）

---

## 7. 无 ROS2 环境时的源码级语法自检（可选）

x86_64 host（如 WSL）已装 ROS2 Humble 时，直接按 §5.3 原生完整编译链接即可，无需本节省略。
仅当你在**没有 ROS2**（或其头文件/消息包）的机器上想先做最轻量的 `-fsyntax-only` 语法自检
（只查语法、不链接）时使用：

```bash
INCS=$(find /opt/ros/humble/include -mindepth 1 -maxdepth 1 -type d \
       | grep -vE "/(dds|idl)$" | sed "s|^|-I|" | tr "\n" " ")
g++ -std=c++17 -fsyntax-only \
    $INCS \
    -I<ws>/install/com_interfaces/include \
    -I<ws>/src/berxel_camera_driver/include \
    -I<ws>/src/berxel_camera_driver/SDK/Include \
    <ws>/src/berxel_camera_driver/src/camera_driver_node.cpp
```

> 需 host 有 ROS2 Humble 头文件及已构建的 `com_interfaces` 消息头。
> 必须 `-std=c++17`（ROS2 Humble 依赖 C++17）；`grep -vE "/(dds|idl)$"` 剔除会劫持
> 系统头文件的 ROS include 子目录（`dds/features.h`、`idl/string.h`）。

---

## 8. 与 camera_driver 的一致性

| 维度 | camera_driver (Orbbec) | berxel_camera_driver (本包) | 一致性 |
|---|---|---|---|
| 节点名 / 可执行 | `camera_driver_node` | `camera_driver_node` | ✅ |
| 状态机 / 复位重连 | 相同 | 相同 | ✅ |
| 多相机 `camera_id_sn_map` | 支持 | 支持（按 SN 逐台打开 Berxel 设备） | ✅ |
| 发布话题 | `/fusion/synced_robot_vision`、`/vision/rgb_raw`、`/SafeNode/Heartbeat`、`/SafeNode/status` | 相同 | ✅ |
| 订阅话题 | `/Control/MotorInfo`、`/Control/HeadPose` | 相同 | ✅ |
| Action | `/vision/Check_self`、`/vision/Reset` | 相同 | ✅ |
| 参数集 | color/depth 分辨率帧率、对齐、点云、时间同步、锁 | 相同（`align_mode` 默认 HW，Berxel 仅硬件注册） | ✅ |
| launch 参数 | `camera_driver.launch.py` | 同名参数（另支持 `config_file:=`） | ✅ |
| 底层数据源 | Orbbec SDK | **Berxel Hawk SDK（本包 `SDK/`）** | 仅此处不同 |

---

## 9. 已知限制与注意事项

- **分辨率/帧率**：最终受固件支持的模式约束。若请求模式不在 `getSupportFrameModes` 内，
  节点会回退到固件支持的模式并打印告警（`getCurrentFrameMode` 回读实际值）。
- **D2C 对齐**：Berxel SDK 只提供硬件深度→彩色注册，`align_mode` 不支持真正的软件重投影；
  `align_target_stream=DEPTH` 会告警并回退 `COLOR`。
- **跨进程锁**：与 Orbbec 驱动同机运行互不冲突；请勿让两个驱动进程同时打开同一台 Berxel 设备
  （`bypass_lock=false` 时会加跨进程锁防护）。
- **深度数值**：深度流为原始 raw 值（非米制），内部按像素类型换算后生成米制点云；
  深度对齐到彩色时分辨率等于彩色流分辨率。
- **仅实现本需求能力**：与官方 `RosSDKNode/berxel_camera_ros2` 思路一致
  （回调 + 有界队列 + 消费者线程 + 手写点云反投影 + 按分辨率缩放内参），
  本包未接入 IR/lightIR/GMSL 等扩展。
- **USB 权限（首次必做）**：`ret=-1` 通常是 SDK 枚举到 0 台设备——先确认 `lsusb` 能看到相机，
  再按 §6.1 安装随包 `udev/berxel-usb.rules` 并拔插；不要同时让两个进程打开同一台 Berxel 设备。
- **USB 供电 / 自动挂起（RK3576 已遇到）**：两条流成功启动、状态 IDLE 后约 1 秒设备立刻
  `LEFT` 再以新 DEV 号 `ARRIVE` → 帧数恒为 0。这是物理层掉线，常见于**无源 USB hub**供电不足
  （本机经 Genesys 0x1a40 无源 hub 接入，SDK 日志亦提示检查 hub 供电）。udev 规则已加
  `ATTR{power/control}="on"` 关闭相机 USB 自动挂起；仍掉线请直连主板 USB 口或换带供电 hub，
  并以 `sudo dmesg` 抓掉线原因（见 §6）。
