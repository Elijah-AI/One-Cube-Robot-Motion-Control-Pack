/*
 * 第 2 层：单个德晟舵机的直接控制。
 * 寄存器含义见 ds485_protocol.h；收发经第 1 层 Rs485Port。
 */
#include "ds485_driver/ds485_servo.h"

#include <algorithm>
#include <stdio.h>
#include <string.h>

using namespace ds485;

// 格式化成 std::string 的小工具
static std::string Format(const char *fmt, int a = 0, int b = 0, int c = 0)
{
    char text[200];
    snprintf(text, sizeof(text), fmt, a, b, c);
    return text;
}

// 这些寄存器被 Probe 缓存，被改写后需要重新 Probe
static bool Is_Probe_Reg(uint8_t addr)
{
    return addr == DS485_REG_SYS_CONFIG || addr == DS485_REG_MIN_ANGLE ||
           addr == DS485_REG_MAX_ANGLE || addr == DS485_REG_MODE ||
           addr == DS485_REG_SPEED_LIMIT;
}

Ds485Servo::Ds485Servo(Rs485Port *port, uint8_t id) : transactions_(port)
{
    port_ = port;
    id_ = id;
    probed_ = false;
    write_ack_ = false;
    min_angle_ = 0;
    max_angle_ = 0;
    speed_limit_ = 0;
    mode_ = 0;
}

void Ds485Servo::SetTiming(int gap_ms, int timeout_ms)
{
    transactions_.Timing(std::max(0, gap_ms), std::max(2, timeout_ms));
}

void Ds485Servo::SetError(const std::string &text)
{
    last_error_ = Format("舵机 ID %d：", id_) + text;
}


// ======================= 收发 =======================

bool Ds485Servo::ReadReg(uint8_t addr, uint8_t *data, int expect_len,
                         const uint8_t *request, int request_len, int *got_len)
{
    if (got_len)
    {
        *got_len = 0;
    }
    auto response =
        transactions_.Exchange(id_, DS485_CMD_READ, addr, request, request_len, expect_len, true);
    last_tx_ = response.tx;
    last_rx_ = response.rx;
    if (!response.ok)
    {
        protection_ready_ = false;
        SetError(Format("读取 0x%02X 失败 ", addr) + response.error);
        return false;
    }
    if (response.size == 6)
    {
        uint8_t flags = response.frame[4];
        if (flags & DS485_STATE_PROTECTION_MASK)
        {
            ObserveProtection(flags);
        }
        SetError(Format("读取 0x%02X 收到短应答 0x%02X（", addr, flags) + FaultText(flags) + "）");
        return false;
    }
    const int n = response.size - 7;
    memcpy(data, response.frame.data() + 6, n);
    if (got_len)
    {
        *got_len = n;
    }
    if (addr == DS485_REG_STATE && n == 2)
    {
        ObserveProtection(data[0]);
    }
    return true;
}

bool Ds485Servo::WriteReg(uint8_t addr, const uint8_t *data, int len, bool *sent, uint8_t *ack_status)
{
    if (sent)
    {
        *sent = false;
    }
    if (ack_status)
    {
        *ack_status = 0;
    }
    if (!probed_ && !Probe())
    {
        return false;
    }
    auto response = transactions_.Exchange(id_, DS485_CMD_WRITE, addr, data, len, 0, write_ack_);
    last_tx_ = response.tx;
    last_rx_ = response.rx;
    if (sent)
    {
        *sent = response.sent;
    }
    if (response.sent)
    {
        if (addr == DS485_REG_INTERPOLATE || addr == DS485_REG_MOTOR_PWM)
        {
            target_sent_ms_ = response.sent_ms;
        }
        if (addr == DS485_REG_USER_SAVE || addr == DS485_REG_RECOVERY ||
            (addr >= DS485_REG_VOLTAGE_TRIP && addr <= DS485_REG_STALL_TRIP))
        {
            protection_ready_ = false;
        }
        if (Is_Probe_Reg(addr))
        {
            probed_ = false;
        }
    }
    if (!response.ok)
    {
        protection_ready_ = false;
        SetError(Format("写入 0x%02X ", addr) + response.error + (response.sent ? "，执行结果未知" : ""));
        return false;
    }
    if (!write_ack_)
    {
        return true;
    }
    const uint8_t status = response.frame[4];
    if (ack_status)
    {
        *ack_status = status;
    }
    ObserveProtection(status);
    if (status & 0x60)
    {
        if (sent)
        {
            *sent = false;
        }
        SetError(Format("写入 0x%02X 被拒绝，短应答 0x%02X（", addr, status) + FaultText(status) + "）");
        return false;
    }
    if (status & DS485_STATE_PROTECTION_MASK)
    {
        SetError(Format("写入 0x%02X 时舵机处于保护状态 0x%02X（", addr, status) + FaultText(status) +
                 "），指令可能已执行");
        return false;
    }
    return true;
}

bool Ds485Servo::ReadU8(uint8_t addr, uint8_t *value)
{
    uint8_t data[DS485_FRAME_MAX];
    if (!ReadReg(addr, data, 1))
    {
        return false;
    }
    *value = data[0];
    return true;
}

bool Ds485Servo::ReadU16(uint8_t addr, uint16_t *value)
{
    uint8_t data[DS485_FRAME_MAX];
    if (!ReadReg(addr, data, 2))
    {
        return false;
    }
    *value = GetU16(data);
    return true;
}

bool Ds485Servo::ReadI16(uint8_t addr, int16_t *value)
{
    uint8_t data[DS485_FRAME_MAX];
    if (!ReadReg(addr, data, 2))
    {
        return false;
    }
    *value = GetI16(data);
    return true;
}

bool Ds485Servo::ReadI32(uint8_t addr, int32_t *value)
{
    uint8_t data[DS485_FRAME_MAX];
    if (!ReadReg(addr, data, 4))
    {
        return false;
    }
    *value = GetI32(data);
    return true;
}

bool Ds485Servo::WriteU8Check(uint8_t addr, uint8_t value)
{
    uint8_t data[1] = {value};
    if (!WriteReg(addr, data, 1))
    {
        return false;
    }
    uint8_t actual = 0;
    if (!ReadU8(addr, &actual))
    {
        return false;
    }
    if (actual != value)
    {
        SetError(Format("0x%02X 读回不一致：设置 %d，实际 %d", addr, value, actual));
        return false;
    }
    return true;
}

bool Ds485Servo::WriteU16Check(uint8_t addr, uint16_t value)
{
    uint8_t data[2];
    PutU16(data, value);
    if (!WriteReg(addr, data, 2))
    {
        return false;
    }
    uint16_t actual = 0;
    if (!ReadU16(addr, &actual))
    {
        return false;
    }
    if (actual != value)
    {
        SetError(Format("0x%02X 读回不一致：设置 %d，实际 %d", addr, value, actual));
        return false;
    }
    return true;
}


// ======================= 探测 / 状态 =======================

bool Ds485Servo::Probe()
{
    probed_ = false;

    uint8_t sys_config = 0;
    if (!ReadU8(DS485_REG_SYS_CONFIG, &sys_config))
    {
        return false;
    }
    int16_t min_angle = 0;
    int16_t max_angle = 0;
    if (!ReadI16(DS485_REG_MIN_ANGLE, &min_angle) || !ReadI16(DS485_REG_MAX_ANGLE, &max_angle))
    {
        return false;
    }
    uint8_t mode = 0;
    if (!ReadU8(DS485_REG_MODE, &mode))
    {
        return false;
    }
    // 速度上限：个别固件可能不开放，读不到记为 0（MoveAngle 时不检查上限）
    uint16_t speed_limit = 0;
    if (!ReadU16(DS485_REG_SPEED_LIMIT, &speed_limit))
    {
        speed_limit = 0;
    }

    write_ack_ = (sys_config & 0x08) != 0;
    min_angle_ = min_angle;
    max_angle_ = max_angle;
    mode_ = mode;
    speed_limit_ = speed_limit;
    probed_ = true;
    last_error_.clear();
    return true;
}

bool Ds485Servo::ReadStatus(ServoStatus *status)
{
    if (status == 0)
    {
        return false;
    }
    status->valid = false;
    uint8_t state[DS485_FRAME_MAX];
    const double started = port_->NowMs();
    if (!ReadReg(DS485_REG_STATE, state, 2))
    {
        return false;
    }
    // 一次 0x59 取得同一批运动和热数据。有效时间取首个请求之前，保守地计入两笔事务的延时。
    const uint8_t list[] = {DS485_REG_ANGLE, DS485_REG_SPEED, DS485_REG_CURRENT, DS485_REG_PWM,
        DS485_REG_ANGLE_ERROR, DS485_REG_TEMPERATURE, DS485_REG_VOLTAGE};
    uint8_t data[DS485_FRAME_MAX];
    if (!ReadReg(DS485_REG_MULTI_READ, data, 18, list, sizeof(list)))
    {
        return false;
    }
    status->state = state[0];
    status->moving = (state[0] & DS485_STATE_MOVING) != 0;
    status->fault = (state[0] & DS485_STATE_FAULT_MASK) != 0;
    status->angle = GetI32(data);
    status->speed = GetI16(data + 4);
    status->current = GetI16(data + 6);
    status->pwm = GetI16(data + 8);
    status->position_error = GetI32(data + 10);
    status->temperature = GetI16(data + 14);
    status->voltage = GetI16(data + 16);
    status->sampled_ms = started;
    status->valid = true;
    return true;
}

bool Ds485Servo::ReadInfo(ServoInfo *info)
{
    if (info == 0)
    {
        return false;
    }
    memset(info, 0, sizeof(ServoInfo));
    if (!ReadU8(DS485_REG_SYS_CONFIG, &info->sys_config)) return false;
    if (!ReadI16(DS485_REG_MIN_ANGLE, &info->min_angle)) return false;
    if (!ReadI16(DS485_REG_MAX_ANGLE, &info->max_angle)) return false;
    if (!ReadU8(DS485_REG_MODE, &info->mode)) return false;
    if (!ReadU8(DS485_REG_SWITCH, &info->torque_switch)) return false;
    if (!ReadU16(DS485_REG_PWM_LIMIT, &info->pwm_limit)) return false;
    if (!ReadI16(DS485_REG_TEMPERATURE, &info->temperature)) return false;
    if (!ReadI16(DS485_REG_VOLTAGE, &info->voltage)) return false;
    // 以下寄存器个别固件可能不开放，读不到记为 0，不算失败
    if (!ReadU16(DS485_REG_CURRENT_LIMIT, &info->current_limit)) info->current_limit = 0;
    if (!ReadU16(DS485_REG_SPEED_LIMIT, &info->speed_limit)) info->speed_limit = 0;
    if (!ReadU16(DS485_REG_MAX_PWM, &info->max_pwm)) info->max_pwm = 0;
    info->write_ack = (info->sys_config & 0x08) != 0;
    last_error_.clear();
    return true;
}


// ======================= 常用指令 =======================

bool Ds485Servo::WriteMotionSetting(uint8_t addr, const uint8_t *data, int size)
{
    uint8_t ack = 0;
    bool ok = WriteReg(addr, data, size, nullptr, &ack);
    if (!ok && !CanAutoRecover(ack))
    {
        return false;
    }
    // 保护锁存时，PWM 和锁舵写短应答仍可能带保护位。只在读回正确后准备下一步。
    uint8_t actual[DS485_FRAME_MAX];
    if (!ReadReg(addr, actual, size))
    {
        return false;
    }
    if (memcmp(data, actual, size) != 0)
    {
        SetError(Format("运动准备寄存器 0x%02X 读回不一致", addr));
        return false;
    }
    return true;
}

bool Ds485Servo::MoveAngle(int32_t angle, uint16_t speed, bool *sent, int pwm_limit)
{
    if (sent != 0)
    {
        *sent = false;
    }
    if (!probed_ && !Probe())
    {
        return false;
    }
    if (mode_ != 0 && mode_ != 1)
    {
        SetError(Format("当前控制模式为 %d（电机模式），不能按角度运动；请先人工切回舵机模式", mode_));
        return false;
    }
    if (speed == 0)
    {
        SetError("速度不能为 0");
        return false;
    }
    if (speed_limit_ > 0 && speed > speed_limit_)
    {
        SetError(Format("速度 %d°/s 超过舵机速度上限 0x5D = %d°/s", speed, speed_limit_));
        return false;
    }
    if (angle < -32768 || angle > 32767)
    {
        SetError("目标角度超出协议范围 -3276.8°~3276.7°");
        return false;
    }
    if (min_angle_ != 0 && angle < min_angle_)
    {
        SetError(Format("目标 %d(0.1°) 低于舵机最小限位 %d(0.1°)", angle, min_angle_));
        return false;
    }
    if (max_angle_ != 0 && angle > max_angle_)
    {
        SetError(Format("目标 %d(0.1°) 高于舵机最大限位 %d(0.1°)", angle, max_angle_));
        return false;
    }

    if (pwm_limit < -1 || pwm_limit > DS485_PWM_MAX)
    {
        SetError("运行时 PWM 上限须为 0~1000");
        return false;
    }
    if (!EnsureProtection())
    {
        return false;
    }
    // 1. 仅允许新指令交给已确认自动解除的固件；硬件、协议故障或混合故障仍拒绝。
    uint8_t state[DS485_FRAME_MAX];
    if (!ReadReg(DS485_REG_STATE, state, 2))
    {
        return false;
    }
    bool latched = (state[0] & DS485_STATE_FAULT_MASK) != 0;
    if (latched && !CanAutoRecover(state[0]))
    {
        SetError(Format("舵机保护状态 0x%02X（", state[0]) + FaultText(state[0]) + "），未下发运动");
        return false;
    }

    // 2. 夹爪命令先确认本次运行 PWM；调试运动沿用已有上限。
    if (pwm_limit >= 0)
    {
        uint8_t pwm[2];
        PutU16(pwm, static_cast<uint16_t>(pwm_limit));
        if (!WriteMotionSetting(DS485_REG_PWM_LIMIT, pwm, 2))
        {
            return false;
        }
    }
    uint8_t torque_switch = 0;
    if (!ReadU8(DS485_REG_SWITCH, &torque_switch))
    {
        return false;
    }
    if (torque_switch != DS485_SWITCH_HOLD)
    {
        const uint8_t hold = DS485_SWITCH_HOLD;
        if (!WriteMotionSetting(DS485_REG_SWITCH, &hold, 1))
        {
            return false;
        }
    }

    // 3. 定速绝对角度 0x66：[角度 Int16, 0.1°][速度 Uint16, °/s]
    uint8_t payload[4];
    PutI16(payload, (int16_t)angle);
    PutU16(payload + 2, speed);
    uint8_t motion_ack = 0;
    bool accepted = WriteReg(DS485_REG_MOVE_SPEED, payload, 4, sent, &motion_ack);
    if (!accepted && !(latched && CanAutoRecover(motion_ack)))
    {
        return false;
    }
    if (latched)
    {
        // 0x66 只发一次。即使固件关闭写应答，也要由状态回读确认保护已解除。
        if (!ReadReg(DS485_REG_STATE, state, 2))
        {
            return false;
        }
        if (state[0] & DS485_STATE_FAULT_MASK)
        {
            SetError(Format("新目标已发送，舵机仍在保护状态 0x%02X（", state[0]) + FaultText(state[0]) + "）");
            return false;
        }
        if (!accepted || !write_ack_)
        {
            // 短应答仍带保护位或无写应答时，状态清零只能证明解除；还需从同一读帧核对实际目标。
            const uint8_t regs[] = {DS485_REG_ANGLE, DS485_REG_ANGLE_ERROR};
            uint8_t feedback[DS485_FRAME_MAX];
            if (!ReadReg(DS485_REG_MULTI_READ, feedback, 8, regs, 2))
            {
                return false;
            }
            int64_t actual = GetI32(feedback);
            int32_t error = GetI32(feedback + 4);
            if (actual + error != angle)
            {
                SetError("保护已解除，但读回的位置和偏差未确认本次目标");
                return false;
            }
        }
    }
    last_error_.clear();
    return true;
}

bool Ds485Servo::SetSwitch(uint8_t mode)
{
    if (mode > DS485_SWITCH_HOLD)
    {
        SetError("扭矩开关只能是 0 释放 / 1 刹车 / 2 锁舵");
        return false;
    }
    return WriteU8Check(DS485_REG_SWITCH, mode);
}

bool Ds485Servo::SetPwmLimit(uint16_t pwm)
{
    if (pwm > DS485_PWM_MAX)
    {
        SetError("PWM 上限范围为 0~1000");
        return false;
    }
    return WriteU16Check(DS485_REG_PWM_LIMIT, pwm);
}

bool Ds485Servo::SetSpeedLimit(uint16_t speed_dps)
{
    if (speed_dps == 0)
    {
        SetError("速度上限不能为 0（为 0 时舵机无法运动）");
        return false;
    }
    if (!WriteU16Check(DS485_REG_SPEED_LIMIT, speed_dps))
    {
        return false;
    }
    // WriteReg 把 probed_ 清掉了；这里只改了速度上限，直接更新缓存即可
    return Probe();
}

bool Ds485Servo::SetCurrentLimit(uint16_t current_ma)
{
    return WriteU16Check(DS485_REG_CURRENT_LIMIT, current_ma);
}

bool Ds485Servo::InspectProtection(std::string *registers)
{
    // 0x3E/3F 的三字节和四字节回报均保留原值，便于按型号审计。
    const uint8_t addresses[] = {
        0x13, 0x14, 0x1C, 0x32, 0x33, 0x34, 0x3D, 0x3E, 0x3F, 0x5A, 0x5B, 0x5C, 0x5D};
    const int widths[] = {2, 2, 2, 1, 1, 1, 2, 0, 0, 1, 2, 2, 2};
    registers->clear();
    bool valid = true;
    for (size_t i = 0; i < sizeof(addresses); ++i)
    {
        uint8_t data[DS485_FRAME_MAX];
        int n = 0;
        bool ok = ReadReg(addresses[i], data, widths[i], nullptr, 0, &n);
        if (!widths[i] && n != 3 && n != 4)
        {
            ok = false;
        }
        *registers += Format(" 0x%02X=", addresses[i]);
        *registers += ok ? "[" + ToHex(data, n) + "]" : "unavailable";
        if (!ok)
        {
            valid = false;
        }
    }
    return valid;
}

bool Ds485Servo::PrepareTrajectory(uint16_t pwm, uint16_t current, uint16_t speed)
{
    if (!Probe() || !EnsureProtection())
    {
        return false;
    }
    if (mode_ == 2 || mode_ > 2)
    {
        SetError("MODE_MISMATCH");
        return false;
    }
    if (mode_ != 1 && !WriteU8Check(DS485_REG_MODE, 1))
    {
        return false;
    }
    if (!probed_ && !Probe())
    {
        return false;
    }
    if (mode_ != 1 || !speed || !speed_limit_ || speed > speed_limit_)
    {
        SetError("SPEED_OR_MODE_INVALID");
        return false;
    }
    // 只用已标定的运行时限制。检查设备的存储上限，先限流限 PWM，再锁舵。
    uint16_t max_pwm = 0, max_current = 0;
    if (!ReadU16(DS485_REG_MAX_PWM, &max_pwm) || !ReadU16(0x14, &max_current))
    {
        return false;
    }
    if (!pwm || pwm > max_pwm || !current || current > max_current)
    {
        SetError("RUNTIME_LIMIT_EXCEEDS_DEVICE");
        return false;
    }
    if (!SetCurrentLimit(current) || !SetPwmLimit(pwm))
    {
        return false;
    }
    uint8_t torque = 0;
    if (!ReadU8(DS485_REG_SWITCH, &torque))
    {
        return false;
    }
    return torque == DS485_SWITCH_HOLD || SetSwitch(DS485_SWITCH_HOLD);
}

bool Ds485Servo::WriteMotorPwm(int16_t pwm, bool *sent)
{
    if (pwm < -DS485_PWM_MAX || pwm > DS485_PWM_MAX)
    {
        SetError("PWM_OUT_OF_RANGE");
        if (sent)
        {
            *sent = false;
        }
        return false;
    }
    uint8_t data[2];
    PutI16(data, pwm);
    bool submitted = false;
    uint8_t status = 0;
    const bool ok = WriteReg(DS485_REG_MOTOR_PWM, data, sizeof(data), &submitted, &status);
    if (sent)
    {
        // 原接口在舵机明确拒绝时清除 sent。新服务分别报告传输受理和协议执行结果。
        *sent = submitted || (status & 0x60) != 0;
    }
    return ok;
}

bool Ds485Servo::WriteInterpolated(int32_t target)
{
    if (!probed_ || mode_ != 1 || target < -32768 || target > 32767 ||
        (min_angle_ && target < min_angle_) || (max_angle_ && target > max_angle_))
    {
        SetError("INTERPOLATION_TARGET_INVALID");
        return false;
    }
    uint8_t data[2];
    PutI16(data, static_cast<int16_t>(target));
    return WriteReg(DS485_REG_INTERPOLATE, data, 2);
}
