# One-Cube-Robot-Motion-Control-Pack

ROS 2 Humble 全身运控、双臂运动、视觉与任务编排工作区，源码来自 `ylf_wbc`。

- **[2026-11-13 SDK 限位与示教合并版](src/motion_control_pkg/doc/SDK_LIMITS_AND_TEACH_RELEASE.md)**：双臂位置限位统一到 SDK，规划/WBC/安全监控/IK 重解使用相同范围；示教切换采用 5 s、0.05 rad、0.03 rad/s，并保留整机清错恢复和退出修补。
- [示教开启、关闭与整机清错操作](src/motion_control_pkg/doc/TEACH_AND_RECOVERY.md)
- **[运控指令与参数完整说明](doc/MOTION_COMMANDS_AND_PARAMETERS.md)**：全部运控 Action 字段、部件模式、MoveJ/MoveP/MoveL、腰腿协同、身体升降/倾角、夹爪/吸盘、启动参数、WBC、碰撞、反馈和任务 YAML。
- [工作区与专题文档](doc/README.md)
- [任务节点命令](doc/TASK_NODE_COMMANDS.md)
- [双吸盘刚性搬箱](doc/RIGID_GRASP.md)：吸盘竖直向下、接触点同高、双手相对位姿和间距保持。

当前发布保留功能源码、接口、配置、模型、必要的第三方依赖和使用示例；保留由 `BUILD_TESTING` 控制的硬件无关回归源码；构建产物、现场测试任务和备份不提交。构建产物与备份不提交。

编译与运行前请先阅读完整手册中的平台依赖、接口版本和实际配置路径说明。

2026-11-04 已同步机器人运控源码及配套 x86 DriverSDK；版本范围、SHA-256 和构建方法见 [机器人同步记录](doc/ROBOT_SYNC_20261104.md)。
