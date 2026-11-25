# 夹爪停止：外部模块发布到 /ds485/gripper/stop
# 在当前位置锁舵（保持夹持力），并结束该侧正在执行的命令（结果为 CANCELED）
uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1
uint8 SIDE_BOTH=2

uint8 side
