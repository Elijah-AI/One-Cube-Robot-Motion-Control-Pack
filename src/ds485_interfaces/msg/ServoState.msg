# 一侧舵机/夹爪的实时状态，节点按 state_rate_hz 周期发布到 ~/state
uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1

uint8 side            # 0 左（A 臂）1 右（B 臂）
bool online           # 本次读取是否成功
string error          # online=false 时的原因
bool busy             # 是否正在执行夹爪命令或调试运动

uint8 state_raw       # 舵机状态字节原值（0x01）
bool moving           # Bit7 正在转动
bool fault            # Bit0~Bit6 任一置位
string fault_text     # 故障位的中文说明

float32 angle_deg     # 舵机当前角度，度
int16 speed_dps       # 当前速度，度/秒
int16 current_ma      # 电流，毫安
int16 pwm             # 当前驱动 PWM，0~1000
bool calibrated       # 夹爪张开/闭合角度是否已标定
uint8 position        # 夹爪位置 0 全开 ~ 255 全闭（未标定时为 0）
