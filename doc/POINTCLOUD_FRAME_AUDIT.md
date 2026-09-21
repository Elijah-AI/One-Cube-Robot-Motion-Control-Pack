# 点云坐标链路与 box 碰撞对象（2026-09-20）

核查对象为本工作区的碰撞点云链路，不以另一个工作区或机器人旧程序代替。

## 结论

相机侧已经进行坐标转换；正确标记为 `base_link` 的点云在运控端没有再次应用相机外参。
原运控函数虽然名为 `pointCloudToModelRoot`，但本模型根为 `base_link`，根帧到根帧的变换是单位阵。
本次明确让单位变换下的有效 XYZ 点直接通过，并以回归测试锁定行为；没有删除其他模型坐标系需要的真实变换。

## 实际数据流

1. `camera_driver_node.cpp::onFrameSet()` 先产生以米为单位的相机点云，再调用 `applyPointCloudTransform()`。
2. 头部相机使用 `M_end * kFixedTransform_0`；底部相机使用 `kFixedTransform_1`。
   成功变换后设置**点云自己的** `header.frame_id=pointcloud_base_frame_id_`，默认 `base_link`。
   头部位姿未到达时，头部点云保持原相机坐标和标签，不能当作已转换的基座点云。
3. `fusion_cb()` 原样复制到 `SyncedRobotVision.camera_rgbd_pointcloud`。
   消息定义中“位于彩色光学坐标系”的旧注释不足以判断当前坐标，必须检查内层点云 header。
4. 运控 `loadPointCloud()` 只取该内层字段；不使用融合消息外层 header 覆盖点云 frame。
5. `pointCloudToModelRoot()` 验证 XYZ 布局，以消息 frame 和模型 FK 求变换。
   单位变换时保留 XYZ；非单位变换时转换一次。未知 frame 拒绝，不猜测、不强行改标签。
6. `ScopedCollisionScene` 以名称 `box` 加入 1 cm 占据 Octree；规划场景退出后移除该对象。

另一个入口是 `MovePose.env_point_cloud`，非空时优先使用，同样遵循上述内层 header 契约。
`MovePose.point_cloud_frame_id` 当前不覆盖这个 header。
`DetectionPoseCloud` 返回目标位姿而不是环境 PointCloud2，因此目标检测库的位姿后处理不在上述碰撞点云通路中。
第三方检测源码存在历史手眼位姿转换代码，且当前链接预编译库；本次没有据此擅自反变换目标位姿，也不把碰撞点云回归当作 NPU 位姿精度验收。

## 仍需满足的输入条件

- XYZ 的实际坐标与 `header.frame_id` 一致；禁止只改标签而不变换坐标。
- 头部姿态和图像采集应由上游正确配对。当前相机使用缓存头部位姿，运控非根帧转换使用规划时状态，本次未实现采集时刻插值。
- 只有 `enable_collision_check=2` 导入环境点云；0/1 不导入。
- 对象名 `box` 不等于 ROS 话题名、视觉分类名或坐标系名。
- 仍是规划时单帧环境检查，不是在线动态障碍急停。

## 新增离线回归

- 基座点云在不同关节状态下保持坐标；过滤 NaN。
- 已知带旋转和平移的相机 frame 只转换一次；未知 frame 拒绝。
- 场景中能按 `box` 删除碰撞物；重新加入、作用域退出均正确清理。
- 错误大小端/XYZ 类型拒绝，显式坏点云不回退到话题。
- 当前整机 URDF 下，左/右/双臂三种全身 IK 求解邻近 FK 目标，检查位置、姿态和非参与关节冻结。
- 保留原有“关闭几何碰撞不关闭 COM 保护”、缺云拒绝、COM/ZMP 模型及反馈跟踪回归。

这些都是不驱动硬件的测试，不代表真实相机标定、真实关节反馈或物理避障已验收。
