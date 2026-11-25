# 夹爪命令结果：节点发布到 /ds485/gripper/result，每侧每条命令一条
uint8 RESULT_ARRIVED=1    # 到达目标位置
uint8 RESULT_BLOCKED=2    # 夹到物体（未到目标就停住，保持夹持力）
uint8 RESULT_FAULT=3      # 执行中发现保护，本次命令结束，现场读数见节点日志
uint8 RESULT_TIMEOUT=4    # 超时（已锁舵停在当前位置）
uint8 RESULT_FAILED=5     # 未执行或通信失败（参数错误、未标定、舵机不应答等）
uint8 RESULT_CANCELED=6   # 被停止（/ds485/gripper/stop 或调试服务 set_switch）
uint8 RESULT_PREEMPTED=7  # 被同一侧的新命令替换

uint32 id                 # 对应 GripperCommand.id
uint8 side                # 0 左 1 右
uint8 result
bool success              # result 为 ARRIVED 或 BLOCKED
string message
uint8 final_position      # 结束时的夹爪位置 0~255
float32 final_angle_deg   # 结束时的舵机角度
