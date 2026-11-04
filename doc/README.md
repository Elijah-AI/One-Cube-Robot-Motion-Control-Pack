# ylf_wbc 文档入口

**[运控指令与参数完整说明](MOTION_COMMANDS_AND_PARAMETERS.md)** 是本次发布的主手册，按当前源码整理所有已实现的运控入口、字段、启动参数、使用示例和限制。

## 专题说明

- [机器人源码同步与配套 SDK](ROBOT_SYNC_20261104.md)
- [头部启动回零](HEAD_STARTUP_HOMING.md)
- [腿腰和头部精确到位](PRECISION_ARRIVAL.md)
- [WBC 末端水平约束](TCP_LEVEL_CONSTRAINT.md)
- [全身 DH 参数](WHOLE_BODY_DH_PARAMETERS.md)

- [全身运控与点云接口](MOTION_WBC_USAGE.md)
- [20 关节实测反馈](MEASURED_JOINT_FEEDBACK.md)
- [身体倾角与竖直升降](LEG_BODY_MOTION.md)
- [腿部两关节补偿保持身体朝向](LEG_BODY_ORIENTATION_MOVEJ.md)
- [腰部与手臂 MoveP](WHOLE_BODY_WAIST_ONLY_MOVEP.md)
- [WBC 完成反馈判据](WBC_COMPLETION_FEEDBACK.md)
- [WBC IK 求解预算](WBC_IK_TUNING.md)
- [任务节点命令](TASK_NODE_COMMANDS.md)
- [笔记本两件抓放任务](LAPTOP_ASSEMBLY_TASK.md)
- [笔记本 IK 升降恢复](LAPTOP_IK_LEG_RECOVERY.md)

## 发布内容

保留功能源码、接口、配置、launch、机器人模型和必要第三方依赖。测试源码、测试数据目录、精度测试任务、测试构建目标和测试指南已移出工作区；备份在仓库外保留，不上传。功能示例和正式业务任务保留。

本次清理仅移除 `laptop_dual_slot1_accuracy_test` 的 40011/50011 注册，其余任务顺序和参数保持现状。主配置当前监听 `127.0.0.1:12000`，Modbus 地址转换与 `mbpoll -0` 用法见完整手册。

历史交付/合并记录描述当时的环境和验证结果，不代表当前机器人运行状态。源码上传、构建、安装、重启和实机验收分别确认。
