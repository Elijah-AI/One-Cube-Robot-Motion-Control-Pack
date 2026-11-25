# 夹爪命令：外部模块发布到 /ds485/gripper/command
# 同一侧收到新命令时立即改为执行新命令（旧命令结果为 PREEMPTED）
uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1
uint8 SIDE_BOTH=2

uint32 id             # 命令编号，由发布方自定；结果 /ds485/gripper/result 中原样带回，用于对应
uint8 side            # 0 左 1 右 2 两侧（两侧使用相同参数，各自发布一条结果）
uint8 position        # 0 全开 ~ 255 全闭
uint8 torque          # 0~255，0 = 默认（配置 default_pwm，默认最大扭矩）
uint8 speed           # 0~255，0 = 默认（配置 default_speed，默认最大速度）
uint32 timeout_ms     # 0 = 配置 default_timeout_ms（默认 10000）
