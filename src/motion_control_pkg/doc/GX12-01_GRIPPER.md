# GX12-01 右夹爪

标准 bringup 读取 /opt/onecube/motion_control/config/motion_control_config.yaml。将 right_arm_tool 设为 ds485_torque 后，运控在本进程内接入 DS485，共用双臂已连接的 MarvinSDK。左侧吸盘沿用现场配置。

右夹爪参数在 ds485_driver 包的 config/ds485_driver.yaml，来源为 GX12-01 的 ds485_ws 提交 2be30da。右臂 B 的 COM1（channel 2）连接 ID 1、2，方向系数均为 +1，采用 multi_id 同帧写入，启动确认窗口为 200 ms。工具 TCP 和负载标定由现场运控配置提供。

右夹爪使用 /ds485/servo/set_torque（ds485_interfaces/srv/ServoSetTorque），side 填 1。pwm 范围为 -1000 到 1000。duration_ms 大于零表示定时脉冲，零表示持续输出。同步广播没有协议应答，启动确认看 /ds485/gripper/health 的 PWM_START_CONFIRMED 和两台舵机各自的反馈。停止时发送 pwm 为零的请求，并检查 stop_confirmed。PWM 正负与机械开合的对应关系需要在现场确认。

/Control/control_gripper 保留位置 Action 的参数、目标和结果语义。右端配置为 ds485_torque 时，针对右侧或双侧的位置目标会被拒绝。
