# ylfrobot 批量依赖安装

适用于已有 ROS 2 和相应 apt 软件源的 Ubuntu/Debian 环境，当前清单依据本工作区 Ubuntu 22.04 / Humble 源码整理。无需额外 Python 库即可运行。

## 使用

以普通用户执行，安装时程序内部对系统操作使用 sudo：

```bash
cd ~/ylfrobot
python3 tools/dependencies/install_deps.py           # 默认仅预览，不联网、不安装
python3 tools/dependencies/install_deps.py list      # 查看清单
python3 tools/dependencies/install_deps.py install   # 实际批量安装
```

程序安装补充 apt 库、按需初始化 rosdep、更新索引、扫描源码中的 package.xml（优先使用 `src/`；无 `src/` 时仅扫描仓库根目录下直接包含 `package.xml` 的包目录，排除 build/install/log/.deps_venv）、安装 ROS 依赖，最后执行 rosdep check。使用 rosdep 的 --ignore-src 排除工作区内源码包；遵循 rosdep 的目录发现规则，源码中的 COLCON_IGNORE/CATKIN_IGNORE 需保持正确。未知依赖或安装失败立即返回非零状态，不报告全部成功。详细日志在 `log/dependencies/`，修复网络/源/包名后重新运行即可；不使用强制重装或全系统 upgrade。

默认发行版取 ROS_DISTRO，未设置时使用 humble。换工作区或发行版时，全局参数放在子命令之前：

```bash
python3 tools/dependencies/install_deps.py --workspace /path/to/ros2_ws --ros-distro humble install
python3 tools/dependencies/install_deps.py install --dry-run
python3 tools/dependencies/install_deps.py --no-update install
```

`--no-update` 要求已有可用 apt 与 rosdep 索引；它只跳过索引更新，安装缺失包仍可能联网。清单在其他 Ubuntu/ROS 版本及 ARM64 的可用性由目标机器软件源决定，缺失包会明确失败，不会擅自改源或源码编译。预览只输出计划，不验证远端包是否可下载。

## 后续添加库

```bash
# 系统库，可一次添加多个；重复项不会重复保存
python3 tools/dependencies/install_deps.py add apt libboost-all-dev libfmt-dev
# Python 库，支持版本约束
python3 tools/dependencies/install_deps.py add pip 'transforms3d==0.4.2'
# 厂家提供的匹配板端 Python ABI 的 wheel
python3 tools/dependencies/install_deps.py add pip /path/to/rknn_toolkit_lite2-version-cp310-cp310-linux_aarch64.whl
# 标准 rosdep key（新 ROS 包优先在自己的 package.xml 中声明）
python3 tools/dependencies/install_deps.py add rosdep diagnostic_updater
python3 tools/dependencies/install_deps.py install
```

也可直接编辑同目录 `dependencies.json`：apt、pip、rosdep 各自为字符串数组；删除条目不会卸载已安装库。apt 支持 `{ros_distro}` 占位符及 `包名=版本`。pip 的版本约束需加引号；本地 wheel 相对路径以清单所在目录为基准，绝对路径换机器后需调整。清单只应填写可信包源，不保存凭据。

Python 扩展库安装到工作区 `.deps_venv`，继承系统 Python 包以使用 apt 的 numpy/OpenCV 等，不执行 sudo pip。新增 pip 包后，编译及运行 Python 节点使用：

```bash
source /opt/ros/humble/setup.bash
source ~/ylfrobot/.deps_venv/bin/activate
# 如已有节点由其他 Python 解释器构建，需要在此环境重新构建对应 Python 包。
```

## 覆盖范围与边界

补充清单覆盖当前 CMake 中的 Eigen、URDFDOM、FCL、CCD、OctoMap、OMPL、Pinocchio、Coal、Ruckig、JSON、YAML、Modbus、UUID，以及相机/视觉公共依赖。并按用户要求加入 `libpcl-dev`、`ros-{ros_distro}-pcl-conversions`、`ros-{ros_distro}-pcl-ros`，支持点云处理及 ROS 消息转换。依赖清单同步自 ros2_ws；安装 PCL 本身不代表点云碰撞功能验证通过。

其他 package.xml 依赖由 rosdep 动态处理，后续新增 ROS 包无需改程序。当前 XML 漏声明的外部库通过补充清单安装；现有包文件未改动。

`skip_rosdep` 中保留源工作区视觉包的两个历史错误声明（本分支没有对应包时不生效），每项附有原因，程序每次显示。新增未知 key 不会自动跳过。新包应补齐正确 package.xml 声明。

`manual` 列出 RKNN、厂家 ARM64 驱动、已有二进制和可选硬件解码依赖；这些不能通过公共包名确定正确版本。需要获得厂家 SDK/wheel 并满足其驱动要求。退出码 0 表示所列公共安装步骤和 rosdep 检查成功，**不包含 manual 项、工作区编译、二进制 ABI 或硬件验证**。本次交付只完成离线测试，未运行实际安装。

rosdep 行为参考：[官方命令实现与帮助](https://github.com/ros-infrastructure/rosdep/blob/master/src/rosdep2/main.py)。
