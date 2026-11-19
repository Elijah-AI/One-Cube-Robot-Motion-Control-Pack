#ifndef DS485_DRIVER_DS485_SERVO_H
#define DS485_DRIVER_DS485_SERVO_H

/*
 * 第 2 层：单个德晟舵机的直接控制（寄存器读写 + 常用指令）。
 * 只依赖第 1 层 Rs485Port，不依赖 SDK/ROS。
 *
 * 用法：
 *   Ds485Servo servo(&port, 1);       // 总线 + 舵机 ID
 *   servo.Probe();                    // 读写应答设置、限位、模式、速度上限（失败可再次调用）
 *   servo.ReadStatus(&status);        // 状态 + 角度/速度/电流/PWM
 *   servo.MoveAngle(250, 100);        // 以 100°/s 转到 25.0°
 *   servo.SetSwitch(DS485_SWITCH_HOLD);
 *
 * 安全约定（与 ds485_servo.py 一致）：
 *   - 写指令不自动重发：应答超时时结果未知，重发可能重复执行。
 *   - 配置自动解除后，仅新的运动请求可触发固件重新判断四类异常。硬件或协议故障仍阻止运动。
 *   - 保护参数按差异写入并读回；调试接口 WriteReg 保留原始应答语义。
 */

#include <cstdint>
#include <functional>
#include <string>

#include "ds485_driver/ds485_protocol.h"
#include "ds485_driver/rs485_port.h"
#include "ds485_driver/serial_transaction.h"

// 实时状态（ReadStatus 填写，共 2 次通信）
struct ServoStatus
{
    uint8_t state;      // 状态字节原值
    bool moving;        // Bit7 正在转动
    bool fault;         // Bit0~Bit6 任一置位
    int32_t angle;      // 当前角度，0.1°
    int16_t speed;      // 当前速度，°/s
    int16_t current;    // 电流，mA
    int16_t pwm;        // 当前驱动 PWM，0~1000
    int32_t position_error = 0;
    int16_t temperature = 0, voltage = 0;
    double sampled_ms = 0;
    bool valid = false;
};

// 设备信息（ReadInfo 填写，逐个寄存器读取，较慢，调试用）
struct ServoInfo
{
    uint8_t sys_config;       // 0x11
    bool write_ack;           // 0x11 Bit3
    int16_t min_angle;        // 0x19，0.1°，0 不限
    int16_t max_angle;        // 0x1A，0.1°，0 不限
    uint8_t mode;             // 0x5A：0 自动 1 舵机 2 电机
    uint8_t torque_switch;    // 0x64：0 释放 1 刹车 2 锁舵 3 预启动
    uint16_t pwm_limit;       // 0x5B
    uint16_t current_limit;   // 0x5C，mA
    uint16_t speed_limit;     // 0x5D，°/s
    uint16_t max_pwm;         // 0x13（存储值）
    int16_t temperature;      // 0x4A，℃
    int16_t voltage;          // 0x4B，mV
};

// 每侧独立。阈值 -1 保留舵机现值；毫秒数必须是 100 的整数倍。
struct ServoProtectionConfig
{
    bool auto_recover = true;
    int64_t voltage_high_v = -1;
    int64_t voltage_low_v = -1;
    int64_t temperature_trip_c = -1;
    int64_t temperature_hysteresis_c = -1;
    int64_t current_ma = -1;
    int64_t current_detect_ms = -1;
    int64_t stall_pwm = -1;
    int64_t stall_detect_ms = -1;
};

class Ds485Servo
{
public:
    /**
     * @param port  第 1 层总线，生命周期由调用者管理
     * @param id    舵机 ID 0~250
     */
    Ds485Servo(Rs485Port *port, uint8_t id);

    /**
     * @brief 总线时序
     * @param gap_ms      每次发送前等待（半双工总线留出切换时间），默认 10
     * @param timeout_ms  单次应答超时，默认 300
     */
    void SetTiming(int gap_ms, int timeout_ms);

    uint8_t Id() const { return id_; }
    Rs485Port *Port() const { return port_; }
    ds485::SerialTransactionManager &Transactions() { return transactions_; }
    double LastTargetSendMs() const { return target_sent_ms_; }
    bool WriteInterpolated(int32_t target);
    bool PrepareTrajectory(uint16_t pwm, uint16_t current, uint16_t speed);
    bool InspectProtection(std::string *registers);

    // ================= 探测 =================

    /**
     * @brief 读取 0x11(写应答设置)、0x19/0x1A(角度限位)、0x5A(模式)、0x5D(速度上限)
     *        并缓存。写指令和 MoveAngle 依赖这些值，第一次使用前会自动调用。
     */
    bool Probe();
    bool IsProbed() const { return probed_; }
    bool WriteAckEnabled() const { return write_ack_; }
    bool WriteMotorPwm(int16_t pwm, bool *sent = nullptr);

    // 配置时只校验参数；首次探测和新运动入口在该侧总线锁内同步到舵机。
    bool SetProtectionConfig(const ServoProtectionConfig &config);
    bool EnsureProtection();
    void SetFaultLogger(const std::function<void(const std::string &)> &logger) { fault_logger_ = logger; }

    // Probe 缓存的值
    int16_t MinAngle() const { return min_angle_; }
    int16_t MaxAngle() const { return max_angle_; }
    uint16_t SpeedLimit() const { return speed_limit_; }   // 0 表示读不到
    uint8_t Mode() const { return mode_; }

    // ================= 寄存器读写（调试接口）=================

    /**
     * @brief 读寄存器
     * @param addr        寄存器地址
     * @param data        输出数据，至少 DS485_FRAME_MAX 字节
     * @param expect_len  期望数据长度；<=0 表示不检查
     * @param request     读指令附带的参数（只有 0x59 地址列表读需要），可为空
     * @param got_len     实际数据长度，可为空
     */
    bool ReadReg(uint8_t addr, uint8_t *data, int expect_len,
                 const uint8_t *request = 0, int request_len = 0, int *got_len = 0);

    /**
     * @brief 写寄存器。写应答打开时检查短应答状态，关闭时只发送。
     * @param sent  总线接口已受理发送时置 true（Marvin 会先入发送槽）。明确拒绝时清零。
     * @param ack_status  有短应答时输出状态字，无短应答时为 0。配置写入由读回确认。
     */
    bool WriteReg(uint8_t addr, const uint8_t *data, int len, bool *sent = 0, uint8_t *ack_status = 0);

    bool ReadU8(uint8_t addr, uint8_t *value);
    bool ReadU16(uint8_t addr, uint16_t *value);
    bool ReadI16(uint8_t addr, int16_t *value);
    bool ReadI32(uint8_t addr, int32_t *value);
    // 写入后读回，读回不一致返回 false
    bool WriteU8Check(uint8_t addr, uint8_t value);
    bool WriteU16Check(uint8_t addr, uint16_t value);

    // ================= 常用指令 =================

    // 状态字节(0x01) + 角度/速度/电流/PWM(0x59 一次读出)
    bool ReadStatus(ServoStatus *status);
    // 设备信息（逐个读，约 11 次通信）
    bool ReadInfo(ServoInfo *info);

    /**
     * @brief 定速转到绝对角度（0x66）
     *        检查：保护位、模式、舵机角度限位、速度上限；扭矩开关不是锁舵时先打开
     * @param angle  目标角度，0.1°
     * @param speed  速度 °/s，1 ~ 速度上限(0x5D)
     * @param sent   可为空。运动指令已发出时置 true（调用者据此决定失败时是否要锁舵停止）
     */
    // 夹爪可随新目标指定运行时 PWM；-1 沿用舵机当前上限。
    bool MoveAngle(int32_t angle, uint16_t speed, bool *sent = 0, int pwm_limit = -1);

    // 扭矩开关 0x64：0 释放 / 1 刹车 / 2 锁舵（写入后读回）
    bool SetSwitch(uint8_t mode);
    // 运行时 PWM 上限 0x5B，0~1000（协议称“扭矩”）
    bool SetPwmLimit(uint16_t pwm);
    // 运行时速度上限 0x5D，°/s（未实机验证）
    bool SetSpeedLimit(uint16_t speed_dps);
    // 运行时电流上限 0x5C，mA（未实机验证）
    bool SetCurrentLimit(uint16_t current_ma);

    // 最近一次失败原因（中文）
    const std::string &LastError() const { return last_error_; }

    // 最近一次发送/收到的报文（十六进制），调试用
    const std::string &LastTx() const { return last_tx_; }
    const std::string &LastRx() const { return last_rx_; }

private:
    void SetError(const std::string &text);
    void ObserveProtection(uint8_t state);
    bool CanAutoRecover(uint8_t state) const;
    bool WriteMotionSetting(uint8_t addr, const uint8_t *data, int size);

    Rs485Port *port_;
    uint8_t id_;
    ds485::SerialTransactionManager transactions_;
    double target_sent_ms_ = -1;  // 总线线程在 0x69 或 0x6E 交给传输层后更新，用于规划周期和脉冲计时。

    bool probed_;
    bool write_ack_;
    int16_t min_angle_;
    int16_t max_angle_;
    uint16_t speed_limit_;
    uint8_t mode_;

    ServoProtectionConfig protection_;
    bool manage_protection_ = false;  // LoadSide 设置；未传配置的独立调用者沿用原有保护策略。
    // 总线锁内维护。完整读回才置位；配置变动、通信中断或原始写相关寄存器时失效。
    bool protection_ready_ = false;
    bool saves_parameters_ = false;  // 同步时从 0x04 读取；未确认可保存时，每个新目标前复核。
    // 状态读或写短应答发现新保护时记录；正常状态清除。两侧各自去重。
    uint8_t reported_faults_ = 0;
    bool sampling_fault_ = false;  // 仅在首次保护采样期间置位，避免诊断读收到异常短应答时递归。
    std::function<void(const std::string &)> fault_logger_;

    std::string last_error_;
    std::string last_tx_;
    std::string last_rx_;
};

#endif
