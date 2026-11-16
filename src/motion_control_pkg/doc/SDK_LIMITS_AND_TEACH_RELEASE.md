# SDK 位置限位与示教/清错合并版

本版本将 SDK 位置限位统一补丁合入 `.18` 已验证的示教/清错恢复版本。工作区为 `it@192.168.10.18:/home/it/ylf_wbc`。原有独立双臂完成判定改动未混入此次合并。

## 双臂位置限位

以运行安装目录 `share/motion_control_pkg/config/ccs_m6_40.MvKDCfg` 为源，通过真实 SDK `LOADMvCfg` 读取 PNVA，再转换为弧度。左右臂相同：

| 关节 | 下限，度 | 上限，度 |
|---|---:|---:|
| J1 | -178 | 178 |
| J2 | -120 | 120 |
| J3 | -178 | 178 |
| J4 | -145 | 60 |
| J5 | -178 | 178 |
| J6 | -60 | 60 |
| J7 | -90 | 90 |

范围来自 Marvin M6-CCS 使用说明书 V2.2 印刷第 29 页（PDF 第 38 页）。SDK 配置只调整双臂 J1/J3/J5 的六组 ±170° 为 ±178°，其余字节保持一致。

初始化顺序为：加载 URDF 几何 → SDK 配置及双臂初始化 → 将 14 个手臂位置范围安装到规划模型 → 启动 IK 重解 worker → 建立 WBC 和安全监控。规划、关节目标校验、实测安全检查、WBC 状态有效性及 IK 重解使用相同手臂位置范围。worker 收到相同的限位并校验完整模型签名，不能只更换主程序而保留旧 worker/WBC 库。

URDF 文件无需改写。腿、腰、头部仍使用原有 URDF 范围；原有速度、加速度、jerk、J6/J7 耦合限制继续检查。±178°只表示标称位置范围，不代表任意姿态或路径都可执行。

## 示教与清错

- 示教开关默认关闭；每个正常示教切换停稳阶段最多等待 5 s。
- 窗口位移上限 0.05 rad，反馈速度上限 0.03 rad/s；安全监控保留位置差分速度检查。
- 至少 5 个递增设备帧连续跨越 100 ms；反馈新鲜、SDK 低速标志、按钮释放、模式/拖动状态和错误码检查继续有效。
- 正常关闭按“退出拖动 → 确认 → 重新读取当前姿态 → 位置模式确认”执行。
- 保留整机清错后的反馈重新确认、使能后的短暂稳定等待、恢复下使能回退和重复开启拦截修补。
- 清错、普通运动、故障复位和退出运控使用各自原有的停稳条件。

service 名称和字段不变，task_manager 无需因为此合并新增参数。终端环境、参数来源和完整开启/关闭流程见 [TEACH_AND_RECOVERY.md](TEACH_AND_RECOVERY.md)。

```bash
# 在 .18 的 ROS 环境中，先清错并保持使能
ros2 service call /Control/recover_robot com_interfaces/srv/RecoverRobot '{enable_after_clear: true}'
# 成功且 motion_ready=true 后开启左臂示教；target=1 为右臂，target=2 为双臂
ros2 service call /Control/set_teach_mode com_interfaces/srv/SetTeachMode '{target: 0, enable: true}'
# 松开末端按钮、停稳后关闭，检查 success 和 feedback_confirmed 都为 true
ros2 service call /Control/set_teach_mode com_interfaces/srv/SetTeachMode '{target: 0, enable: false}'
```

启动日志应逐轴打印 `Arm position limit source=SDK`，J1/J3/J5 显示 `lower_deg=-178 upper_deg=178`。示教服务反馈应包含 `timeout_ms=5000 max_velocity_rad_s=0.03 max_displacement_rad=0.05`。

## 验证范围

离线覆盖 177°接受、178.001°拒绝、非法限位原子拒绝、14 个手臂关节白名单、IK worker 模型签名和 WBC 限位同步，并执行示教、清错、反馈、关机及规划回归。真实角度边界检查通过离线模型完成，真机不发送这些边界角度。

本次合并的最终构建、安装哈希和真机结果将在验证完成后补入本节。
