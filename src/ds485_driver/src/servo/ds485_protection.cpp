/* 保护配置和故障采样。各入口由调用方串行访问该侧总线。 */
#include "ds485_driver/ds485_servo.h"

#include <cstdio>
#include <cstring>

namespace
{
bool ValidValue(int64_t value, int64_t maximum)
{
    return value >= -1 && value <= maximum;
}

bool ValidDuration(int64_t ms)
{
    return ms == -1 || (ms >= 0 && ms <= 6553500 && ms % 100 == 0);
}

struct RegisterChange
{
    uint8_t address;
    int64_t first;
    int64_t second;
    uint8_t desired[4] = {};
    int read_size = 0;
    int write_size = 0;
    bool changed = false;

    RegisterChange(uint8_t reg, int64_t a, int64_t b) : address(reg), first(a), second(b) {}

    // 先按实读宽度合并 YAML 中指定的字段，保留未管理的值和保留位。
    std::string Prepare(const uint8_t *current, int size)
    {
        read_size = size;
        write_size = size;
        if (address == DS485_REG_RECOVERY && size == 1)
        {
            desired[0] = (current[0] & ~DS485_AUTO_FAULT_MASK) | static_cast<uint8_t>(first);
        }
        else if ((address == DS485_REG_VOLTAGE_TRIP || address == DS485_REG_TEMP_TRIP) && size == 2)
        {
            desired[0] = first < 0 ? current[0] : static_cast<uint8_t>(first);
            desired[1] = second < 0 ? current[1] : static_cast<uint8_t>(second);
            if (address == DS485_REG_VOLTAGE_TRIP && desired[0] <= desired[1])
                return "高压阈值必须大于低压阈值";
            if (address == DS485_REG_TEMP_TRIP && desired[1] > desired[0])
                return "解除温度回差不能超过触发温度";
        }
        else if ((address == DS485_REG_CURRENT_TRIP || address == DS485_REG_STALL_TRIP) &&
                 (size == 3 || size == 4))
        {
            uint16_t threshold = first < 0 ? ds485::GetU16(current) : static_cast<uint16_t>(first);
            uint16_t ticks = size == 3 ? current[2] : ds485::GetU16(current + 2);
            if (second >= 0)
                ticks = static_cast<uint16_t>(second / 100);
            if (size == 3 && ticks > 255)
                return "该固件只读回一字节时间，最多可确认 25500 ms";
            ds485::PutU16(desired, threshold);
            ds485::PutU16(desired + 2, ticks);
            // 厂商测试表第 39–42 行：三字节回读型号的 0x3E 写三字节，0x3F 写四字节。
            if (address == DS485_REG_STALL_TRIP)
                write_size = 4;
        }
        else
        {
            return "不支持的保护寄存器回包长度 " + std::to_string(size);
        }
        changed = std::memcmp(current, desired, size) != 0;
        return {};
    }
};
}  // namespace

bool Ds485Servo::SetProtectionConfig(const ServoProtectionConfig &config)
{
    if (!ValidValue(config.voltage_high_v, 255) || !ValidValue(config.voltage_low_v, 255) ||
        !ValidValue(config.temperature_trip_c, 255) || !ValidValue(config.temperature_hysteresis_c, 255) ||
        !ValidValue(config.current_ma, 65535) || !ValidValue(config.stall_pwm, DS485_PWM_MAX) ||
        !ValidDuration(config.current_detect_ms) || !ValidDuration(config.stall_detect_ms))
    {
        SetError("保护阈值超出协议范围，检测时间须为 100 ms 的整数倍，-1 表示保留现值");
        return false;
    }
    if ((config.voltage_low_v >= 0 && config.voltage_high_v >= 0 &&
         config.voltage_low_v >= config.voltage_high_v) ||
        (config.temperature_trip_c >= 0 && config.temperature_hysteresis_c > config.temperature_trip_c))
    {
        SetError("保护阈值关系错误：高压须大于低压，温度回差不能超过触发温度");
        return false;
    }
    protection_ = config;
    manage_protection_ = true;
    protection_ready_ = false;
    return true;
}

bool Ds485Servo::EnsureProtection()
{
    if (!manage_protection_ || (protection_ready_ && saves_parameters_))
        return true;

    protection_ready_ = false;
    uint8_t save_mode = 0;
    bool have_save_mode = ReadU8(DS485_REG_USER_SAVE, &save_mode);
    saves_parameters_ = have_save_mode && save_mode == 0;

    // 先校验所有待管理寄存器。全部可表示后才写阈值，最后写解除方式。
    RegisterChange changes[] = {
        {DS485_REG_VOLTAGE_TRIP, protection_.voltage_high_v, protection_.voltage_low_v},
        {DS485_REG_TEMP_TRIP, protection_.temperature_trip_c, protection_.temperature_hysteresis_c},
        {DS485_REG_CURRENT_TRIP, protection_.current_ma, protection_.current_detect_ms},
        {DS485_REG_STALL_TRIP, protection_.stall_pwm, protection_.stall_detect_ms},
        {DS485_REG_RECOVERY, protection_.auto_recover ? 0 : DS485_AUTO_FAULT_MASK, -1},
    };
    uint8_t bytes[DS485_FRAME_MAX];
    for (auto &change : changes)
    {
        if (change.first == -1 && change.second == -1)
            continue;
        int size = 0;
        if (!ReadReg(change.address, bytes, 0, nullptr, 0, &size))
            return false;
        std::string reason = change.Prepare(bytes, size);
        if (!reason.empty())
        {
            char address[8];
            std::snprintf(address, sizeof(address), "0x%02X", change.address);
            SetError(std::string(address) + " " + reason);
            return false;
        }
    }
    for (const auto &change : changes)
    {
        if (!change.changed)
            continue;
        bool sent = false;
        bool write_ok = WriteReg(change.address, change.desired, change.write_size, &sent);
        if (!write_ok && !sent)
            return false;
        // 超时或短应答带保护位时可能已执行。只读回一次确认，当前事务绝不重发。
        if (!ReadReg(change.address, bytes, change.read_size))
            return false;
        if (std::memcmp(bytes, change.desired, change.read_size) != 0)
        {
            char message[120];
            std::snprintf(message, sizeof(message), "保护参数 0x%02X 读回不一致，本侧配置未就绪", change.address);
            SetError(message);
            return false;
        }
    }
    protection_ready_ = true;
    last_error_.clear();
    return true;
}

bool Ds485Servo::CanAutoRecover(uint8_t state) const
{
    uint8_t faults = state & DS485_STATE_FAULT_MASK;
    return manage_protection_ && protection_ready_ && protection_.auto_recover &&
           faults != 0 && (faults & ~DS485_AUTO_FAULT_MASK) == 0;
}

void Ds485Servo::ObserveProtection(uint8_t state)
{
    const uint8_t faults = state & DS485_STATE_PROTECTION_MASK;
    if (!fault_logger_ || sampling_fault_ || faults == reported_faults_)
        return;
    reported_faults_ = faults;
    if (faults == 0)
        return;

    // 先保留引发日志的事务；采样请求不能覆盖原始服务的收发帧及故障原因。
    sampling_fault_ = true;
    const std::string tx = last_tx_, rx = last_rx_, error = last_error_;
    const uint8_t regs[] = {DS485_REG_TEMPERATURE, DS485_REG_VOLTAGE, DS485_REG_CURRENT, DS485_REG_PWM};
    uint8_t data[DS485_FRAME_MAX];
    std::string values[4];
    bool complete = ReadReg(DS485_REG_MULTI_READ, data, 8, regs, 4);
    for (int i = 0; i < 4; ++i)
    {
        if (complete)
            values[i] = std::to_string(ds485::GetI16(data + 2 * i));
        else
        {
            int16_t value = 0;
            if (ReadI16(regs[i], &value))
                values[i] = std::to_string(value);
            else
                values[i] = "不可读";
        }
    }
    last_tx_ = tx;
    last_rx_ = rx;
    last_error_ = error;
    sampling_fault_ = false;
    char prefix[64];
    std::snprintf(prefix, sizeof(prefix), "舵机 ID %u 保护状态 0x%02X", id_, state);
    fault_logger_(std::string(prefix) + "（" + ds485::FaultText(state) +
                  "） temperature_C=" + values[0] + " voltage_mV=" + values[1] +
                  " current_mA=" + values[2] + " pwm=" + values[3]);
}
