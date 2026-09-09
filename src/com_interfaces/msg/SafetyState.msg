# 设备安全状态消息（话题: /Safety/device_safety_state）
#
# status: 核心安全状态（task_manager 状态机直接使用）
#   0 = 正常状态
#   1 = 功能受限/停止当前任务
#   2 = 急停状态
int8 status

# robot_state: 整机 FSM 状态（与 safety_monitor 的 SystemFsmState 枚举口径一致，供前置条件 robot_status 使用）
#   0=UNKNOWN  1=INIT  2=SELF_CHECK  3=RESET  4=RESET_DONE  5=IDLE  6=RUN  7=ABNORMAL  8=EMERGENCY
int8 robot_state

# battery_percentage: 电池电量百分比（%），供前置条件 battery_level(field=percentage) 使用
float32 battery_percentage

# battery_voltage: 电池电压（V），供前置条件 battery_level(field=voltage) 使用
float32 battery_voltage

# error_msg: 安全异常描述。例如:""
string error_msg
