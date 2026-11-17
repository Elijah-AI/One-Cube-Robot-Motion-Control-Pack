#ifndef __DS485_GRIPPER_H
#define __DS485_GRIPPER_H

/*
 * 自制夹爪：德晟串口总线智能舵机（第五版协议），经机械臂末端 485 透传控制。
 *
 * 使用方式与 Modbus_Jodell.h 中的 Jodell_Control 类似：
 *   1. Ds485_Config()          配置通道(2=COM1, 3=COM2)和舵机 ID
 *   2. Ds485_Set_Angle_Range() 配置张开/闭合角度（必须按机构标定）
 *   3. Ds485_Init()            读取舵机应答设置、限位、模式，检查配置
 *   4. Ds485_Move_Abs()        与 Jodell 相同的 0~255 位置/力矩/速度接口
 *   5. 周期调用 Ds485_Status() + Ds485_Watch_Update() 判断到位/夹到物体/故障
 *
 * 本文件只依赖 <cstdint>，不依赖 MarvinSDK、ROS。实际收发由 Ds485_Transport
 * 的子类完成：实机用 Ds485_Marvin_Transport（机械臂透传），离线测试用模拟舵机。
 *
 * 协议要点（与 ds485_control/ds485_servo.py 实测一致）：
 *   发送：F9 FF  ID  LEN(3+N)  CMD  ADDR  参数[N]  CHK
 *   应答：F9 F5  ID  LEN(3+N)  CMD  ADDR  数据[N]  CHK     （读应答）
 *         F9 F5  ID  0x02      STATUS     CHK              （写短应答）
 *   CHK = ~(ID 到最后一个参数的和) & 0xFF，多字节数据小端。
 *   协议中的“扭矩”实际是驱动 PWM，0~1000 对应 0%~100%，不是 N·m。
 */

#include <cstdint>

// ---------------- 左右侧编号 ----------------
// 数值与 ControlGripper 的 GRIPPER_LEFT=0 / GRIPPER_RIGHT=1 一致。
// 左侧走机械臂 A 的透传通道，右侧走机械臂 B 的透传通道。
#define DS485_SIDE_LEFT   0
#define DS485_SIDE_RIGHT  1
#define DS485_SIDE_NUM    2

// ---------------- Ds485_Watch_Update 的返回值 ----------------
#define DS485_MOVE_RUNNING  0   // 仍在运动
#define DS485_MOVE_ARRIVED  1   // 已到达目标角度（没有夹到物体）
#define DS485_MOVE_BLOCKED  2   // 未到目标就停住不动（判定为夹到物体）
#define DS485_MOVE_FAULT    3   // 舵机报告保护/故障

// ---------------- 协议常量 ----------------
#define DS485_CMD_READ   0x02
#define DS485_CMD_WRITE  0x03

#define DS485_REG_STATE        0x01  // 状态(Uint8) + 随机数(Uint8)
#define DS485_REG_SYS_CONFIG   0x11  // 系统配置，Bit3=1 表示写指令有短应答
#define DS485_REG_MAX_PWM      0x13  // 已存储的最大 PWM（只读取，不写）
#define DS485_REG_MIN_ANGLE    0x19  // 最小角度限位 Int16，0.1°，0 表示不限
#define DS485_REG_MAX_ANGLE    0x1A  // 最大角度限位 Int16，0.1°，0 表示不限
#define DS485_REG_ANGLE        0x46  // 实际角度 Int32，0.1°
#define DS485_REG_SPEED        0x47  // 实际速度 Int16，°/s
#define DS485_REG_CURRENT      0x48  // 电流 Int16，mA
#define DS485_REG_PWM          0x49  // 当前 PWM Int16，0~1000
#define DS485_REG_TEMPERATURE  0x4A  // 温度 Int16，℃
#define DS485_REG_VOLTAGE      0x4B  // 电压 Int16，mV
#define DS485_REG_COMBINED     0x59  // 组合读取，地址列表随本次读请求发送
#define DS485_REG_MODE         0x5A  // 控制模式 Uint8：0自动 1舵机 2电机
#define DS485_REG_PWM_LIMIT    0x5B  // 运行时 PWM 上限 Uint16，0~1000（掉电恢复）
#define DS485_REG_CURRENT_LIMIT 0x5C // 运行时电流上限 Uint16，mA（掉电恢复）
#define DS485_REG_SPEED_LIMIT  0x5D  // 运行时速度上限 Uint16，°/s（掉电恢复）
#define DS485_REG_SWITCH       0x64  // 扭矩开关 Uint8：0释放 1刹车 2锁舵
#define DS485_REG_MOVE_TIME    0x65  // 绝对角度+时间(ms)
#define DS485_REG_MOVE_SPEED   0x66  // 绝对角度+速度(°/s)

// 扭矩开关取值
#define DS485_SWITCH_RELEASE 0   // 关闭输出，自由释放
#define DS485_SWITCH_BRAKE   1   // 关闭驱动，带刹车阻尼
#define DS485_SWITCH_HOLD    2   // 打开输出，在当前位置锁舵

// 状态字节中的故障位（Bit0~Bit6），Bit7 为“正在转动”
#define DS485_STATE_FAULT_MASK 0x7F
#define DS485_STATE_MOVING     0x80

// 接收缓存大小
#define DS485_RX_BUF_SIZE 512
#define DS485_ERROR_SIZE  200


/*
 * 收发接口。实机用 Ds485_Marvin_Transport，测试用模拟舵机。
 * side: DS485_SIDE_LEFT / DS485_SIDE_RIGHT
 * channel: 机械臂末端通道，2=COM1，3=COM2
 */
class Ds485_Transport
{
public:
    virtual ~Ds485_Transport() {}

    // 清空该侧透传接收缓存。返回 true 表示成功。
    virtual bool Clear(int side) = 0;

    // 发送 size 字节。返回 true 表示已交给透传接口发送。
    virtual bool Send(int side, uint32_t channel, const uint8_t *data, int size) = 0;

    // 读取已收到的数据，最多 max_size 字节。
    // 返回读到的字节数；0 表示暂无数据；负数表示读取失败。
    virtual int Receive(int side, uint32_t channel, uint8_t *data, int max_size) = 0;

    // 等待 ms 毫秒。实机为真实休眠；测试中由模拟舵机推进模拟时间。
    virtual void Sleep_ms(int ms) = 0;
};


/* 夹爪状态。Ds485_Status 填写前半部分，Ds485_Read_Health 填写后半部分。 */
struct Ds485_status
{
    // --- 状态字节 0x01 拆出的标志位 ---
    uint8_t state_raw;        // 原始状态字节
    bool err_voltage;         // Bit0 电压异常保护
    bool err_over_current;    // Bit1 过流保护
    bool err_over_temp;       // Bit2 过温保护
    bool err_stall;           // Bit3 堵转/过载保护
    bool err_hardware;        // Bit4 硬件异常保护
    bool err_checksum;        // Bit5 校验异常或命令未开放
    bool err_cmd_failed;      // Bit6 命令执行失败
    bool moving;              // Bit7 正在转动

    // --- 实时数据（Ds485_Status 读取）---
    int32_t angle;            // 当前角度，单位 0.1°
    int16_t speed;            // 当前速度，°/s
    int16_t current;          // 当前电流，mA
    int16_t pwm;              // 当前驱动 PWM，0~1000
    uint8_t position;         // 按张开/闭合角度映射后的位置 0~255（0 张开，255 闭合）

    // --- 健康数据（Ds485_Read_Health 读取）---
    int16_t temperature;      // 温度，℃
    int16_t voltage;          // 电压，mV
    uint8_t mode;             // 控制模式 0自动 1舵机 2电机
    uint8_t torque_switch;    // 扭矩开关 0释放 1刹车 2锁舵 3预启动
    uint16_t pwm_limit;       // 运行时 PWM 上限 0~1000
};


/* 单侧夹爪的配置，全部通过 Ds485_Set_xxx 函数修改。 */
struct Ds485_config
{
    uint32_t channel;         // 透传通道：2=COM1，3=COM2
    uint8_t  id;              // 舵机 ID 0~250

    int32_t  open_angle;      // position=0（全开）对应角度，0.1°
    int32_t  close_angle;     // position=255（全闭）对应角度，0.1°

    uint16_t max_speed;       // speed=255 对应的速度，°/s
    uint16_t default_speed;   // speed=0 时使用的速度，°/s

    int32_t  tolerance;       // 到位容差，0.1°
    int32_t  still_band;      // 两次采样角度变化不超过该值视为“不动”，0.1°
    int      blocked_samples; // 连续“不动”多少次判定为夹到物体

    int      gap_ms;          // 每次发送前的间隔，ms（半双工总线留出切换时间）
    int      timeout_ms;      // 单次应答超时，ms
    uint16_t default_pwm = 1000; // torque=0 时使用的运行时 PWM 上限
};


class Ds485_Gripper
{
public:
    explicit Ds485_Gripper(Ds485_Transport *transport);
    ~Ds485_Gripper();

    // ================= 配置函数（只改本地参数，不访问总线）=================

    /**
     * @brief 配置透传通道与舵机 ID（对应 Jodell_modbus_config_A/B）
     * @param side     DS485_SIDE_LEFT / DS485_SIDE_RIGHT
     * @param channel  2=COM1，3=COM2
     * @param id       舵机 ID 0~250
     */
    void Ds485_Config(int side, uint32_t channel, uint8_t id);

    /**
     * @brief 配置张开/闭合角度，单位 0.1°。open 可以大于或小于 close。
     *        两者相等视为未标定，Ds485_Init 会报错。
     */
    void Ds485_Set_Angle_Range(int side, int32_t open_angle, int32_t close_angle);

    /**
     * @brief 配置速度换算：speed=255 对应 max_speed(°/s)，speed=0 时用 default_speed(°/s)，配置为 0 时使用读取的 0x5D 上限
     */
    void Ds485_Set_Speed(int side, uint16_t default_speed, uint16_t max_speed);
    void Ds485_Set_Default_PWM(int side, uint16_t pwm);

    /**
     * @brief 配置到位/夹物判断参数
     * @param tolerance        到位容差，0.1°
     * @param still_band       判定“不动”的角度变化，0.1°
     * @param blocked_samples  连续不动次数，达到后判定夹到物体
     */
    void Ds485_Set_Detect(int side, int32_t tolerance, int32_t still_band, int blocked_samples);

    /**
     * @brief 配置总线时序
     * @param gap_ms      每次发送前等待，ms
     * @param timeout_ms  单次应答超时，ms
     */
    void Ds485_Set_Bus_Timing(int side, int gap_ms, int timeout_ms);

    // 读取当前配置（只读）
    const Ds485_config &Ds485_Get_Config(int side) const;

    // ================= 设备操作（访问总线）=================

    /**
     * @brief 初始化：读取写应答设置(0x11)、角度限位(0x19/0x1A)，检查标定角度。
     *        每次新建对象后、发运动指令前调用一次。
     * @return true 成功；false 失败，原因见 Ds485_Last_Error
     */
    bool Ds485_Init(int side);

    /**
     * @brief 与 Jodell_Move_Abs_A/B 相同的接口
     * @param position  0~255，0 全开，255 全闭，中间线性插值
     * @param torque    0~255，换算为 PWM 上限 torque*1000/255；0 表示使用 default_pwm（默认 1000）
     * @param speed     0~255，换算为 speed*max_speed/255 (°/s)；0 表示使用 default_speed
     * @return true 指令已下发；false 失败
     */
    bool Ds485_Move_Abs(int side, uint8_t position, uint8_t torque, uint8_t speed,
                        bool *motion_may_be_active = 0);

    /**
     * @brief 直接按角度运动（定速 0x66）。会检查标定范围与舵机限位。
     * @param angle  目标角度，0.1°
     * @param speed  速度 °/s，必须大于 0
     */
    bool Ds485_Move_Angle(int side, int32_t angle, uint16_t speed, bool *motion_may_be_active = 0);

    /**
     * @brief 设置运行时 PWM 上限（协议称“扭矩”），并读回确认
     * @param pwm  0~1000，对应 0%~100% 驱动。0 时舵机可能无法运动。
     */
    bool Ds485_Set_Torque(int side, uint16_t pwm);

    /**
     * @brief 设置运行时电流上限(0x5C)，mA，并读回确认。掉电后恢复舵机存储值。
     *        注意：该寄存器未在实机上验证。
     */
    bool Ds485_Set_Current_Limit(int side, uint16_t current_ma);

    /**
     * @brief 设置运行时速度上限(0x5D)，°/s，并读回确认。掉电后恢复舵机存储值。
     *        注意：该寄存器未在实机上验证。
     */
    bool Ds485_Set_Speed_Limit(int side, uint16_t speed_dps);

    // 读取实时状态：状态字节、角度、速度、电流、PWM
    bool Ds485_Status(int side, Ds485_status *rt_status);

    // 读取健康数据：温度、电压、模式、扭矩开关、PWM 上限
    bool Ds485_Read_Health(int side, Ds485_status *rt_status);

    // 停止并在当前位置锁舵（0x64=2），会保持夹持力
    bool Ds485_Hold(int side);
    // 关闭驱动，带刹车阻尼（0x64=1）
    bool Ds485_Brake(int side);
    // 关闭输出，自由释放（0x64=0），夹持的物体可能掉落
    bool Ds485_Release(int side);

    // ================= 到位 / 夹物判断 =================

    // 开始新的判断。target 为目标角度（0.1°），start 为当前角度（0.1°）
    void Ds485_Watch_Start(int side, int32_t target, int32_t start);

    /**
     * @brief 每次读到新状态后调用一次
     * @return DS485_MOVE_RUNNING / ARRIVED / BLOCKED / FAULT
     */
    int Ds485_Watch_Update(int side, const Ds485_status &rt_status);

    // 最近一次 Ds485_Move_Abs / Ds485_Move_Angle 的目标角度，0.1°
    int32_t Ds485_Target_Angle(int side) const;

    // ================= 换算工具 =================

    // 角度(0.1°) -> 位置 0~255（超出范围时截断）
    uint8_t Ds485_Angle_To_Position(int side, int32_t angle) const;
    // 位置 0~255 -> 角度(0.1°)
    int32_t Ds485_Position_To_Angle(int side, uint8_t position) const;

    // 最近一次失败原因（中文）
    const char *Ds485_Last_Error(int side) const;

    // 把状态字节中的故障位写成中文，例如 "过流保护、堵转/过载保护"
    static void Ds485_Fault_Text(uint8_t state, char *text, int text_size);

    // 组装一帧指令，返回帧长度。out 至少 256 字节。
    static int Ds485_Build_Frame(uint8_t id, uint8_t cmd, uint8_t addr,
                                 const uint8_t *payload, int payload_len, uint8_t *out);

    // ================= 寄存器级读写（一般不直接使用）=================

    // 读寄存器，要求数据长度正好为 expect_len
    bool Ds485_Read_Reg(int side, uint8_t addr, uint8_t *data, int expect_len);
    // 写寄存器。写应答打开时检查短应答状态；关闭时只发送。
    bool Ds485_Write_Reg(int side, uint8_t addr, const uint8_t *payload, int len, bool *sent = 0);

    bool Ds485_Read_U8(int side, uint8_t addr, uint8_t *value);
    bool Ds485_Read_U16(int side, uint8_t addr, uint16_t *value);
    bool Ds485_Read_I16(int side, uint8_t addr, int16_t *value);
    bool Ds485_Read_I32(int side, uint8_t addr, int32_t *value);

    // 写入后读回，读回不一致返回 false
    bool Ds485_Write_U8_Check(int side, uint8_t addr, uint8_t value);
    bool Ds485_Write_U16_Check(int side, uint8_t addr, uint16_t value);

private:
    bool Read_Reg_With_Request(int side, uint8_t addr,
        const uint8_t *request, int request_len, uint8_t *data, int expect_len);
    bool Check_Side(int side);
    void Set_Error(int side, const char *text);
    bool Send_Frame(int side, uint8_t cmd, uint8_t addr, const uint8_t *payload, int len);
    // 从接收缓存中找出一帧有效应答，找到返回帧长度并拷贝到 frame，找不到返回 0
    int Pop_Reply(int side, uint8_t *frame);
    bool Check_Move_Target(int side, int32_t angle);

    Ds485_Transport *transport_;
    Ds485_config config_[DS485_SIDE_NUM];

    bool initialized_[DS485_SIDE_NUM];     // Ds485_Init 是否成功
    bool write_ack_[DS485_SIDE_NUM];       // 0x11 Bit3：写指令是否有短应答
    int16_t min_limit_[DS485_SIDE_NUM];    // 舵机最小角度限位，0 不限
    int16_t max_limit_[DS485_SIDE_NUM];    // 舵机最大角度限位，0 不限

    // 接收缓存（处理分包、噪声、坏校验）
    uint8_t rx_buf_[DS485_SIDE_NUM][DS485_RX_BUF_SIZE];
    int rx_len_[DS485_SIDE_NUM];

    // 到位/夹物判断
    int32_t watch_target_[DS485_SIDE_NUM];
    int32_t watch_anchor_angle_[DS485_SIDE_NUM]; // 判断“不动”的基准角度
    int watch_settled_[DS485_SIDE_NUM];    // 连续“停止且在容差内”次数
    int watch_still_[DS485_SIDE_NUM];      // 连续“角度不变”次数

    char last_error_[DS485_SIDE_NUM][DS485_ERROR_SIZE];
};

#endif
