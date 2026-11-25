/*
 * 第 2 层：同一总线多台舵机的同步写。格式见协议第 19~20 页及 ds485_servo_group.h。
 */
#include "ds485_driver/ds485_servo_group.h"

#include <cstdio>
#include <cstring>

namespace ds485
{
namespace
{
std::string Address(const char *what, uint8_t addr)
{
    char text[64];
    std::snprintf(text, sizeof(text), "%s 0x%02X ", what, addr);
    return text;
}
}  // namespace

bool ParseSyncWrite(const std::string &text, SyncWrite *mode)
{
    if (text == "multi_id")
    {
        *mode = SyncWrite::MultiId;
        return true;
    }
    if (text == "staged")
    {
        *mode = SyncWrite::Staged;
        return true;
    }
    if (text == "sequential")
    {
        *mode = SyncWrite::Sequential;
        return true;
    }
    return false;
}

const char *Name(SyncWrite mode)
{
    switch (mode)
    {
    case SyncWrite::MultiId:
        return "multi_id";
    case SyncWrite::Staged:
        return "staged";
    case SyncWrite::Sequential:
        return "sequential";
    }
    return "unknown";
}

Ds485ServoGroup::Ds485ServoGroup(const std::vector<Ds485Servo *> &servos, SyncWrite mode)
    : servos_(servos), mode_(mode)
{
    shared_ = !servos_.empty();
    for (auto *servo : servos_)
    {
        if (!servo)
        {
            shared_ = false;
        }
    }
    // 同一总线的隔离窗和迟到包清理只在共用事务管理器时才对所有舵机有效。
    for (size_t i = 1; shared_ && i < servos_.size(); ++i)
    {
        if (&servos_[i]->Transactions() != &servos_[0]->Transactions())
        {
            shared_ = false;
        }
    }
}

void Ds485ServoGroup::ClearStaged(const std::vector<size_t> &staged)
{
    // 协议规定 0x84 之前的其他指令会清除预存。按最保守的理解，只有发给该台的指令才清除，
    // 所以逐台读一次。读失败不再处理：之后的状态读同样会清除，且下一次 0x84 前总会重新预存。
    for (size_t i : staged)
    {
        uint8_t ignored = 0;
        servos_[i]->ReadU8(DS485_REG_SWITCH, &ignored);
    }
}

SyncResult Ds485ServoGroup::Write(uint8_t addr, const std::vector<std::vector<uint8_t>> &data,
                                  const std::vector<bool> *mask, bool best_effort)
{
    SyncResult result;
    if (!shared_ || data.size() != servos_.size() || (mask && mask->size() != servos_.size()))
    {
        result.error = "SYNC_GROUP_INVALID";
        return result;
    }
    if (!MultiWriteAddress(addr))
    {
        result.error = Address("SYNC_ADDRESS_UNSUPPORTED", addr);
        return result;
    }
    std::vector<size_t> chosen;
    size_t width = 0;
    for (size_t i = 0; i < servos_.size(); ++i)
    {
        if (mask && !(*mask)[i])
        {
            continue;
        }
        if (chosen.empty())
        {
            width = data[i].size();
        }
        if (data[i].empty() || data[i].size() != width)
        {
            result.error = "SYNC_DATA_INVALID";
            return result;
        }
        chosen.push_back(i);
    }
    if (chosen.empty())
    {
        result.ok = true;
        return result;
    }

    if (mode_ == SyncWrite::Sequential)
    {
        // 逐台普通写。启动时中途失败立即返回，由调用者停止已经执行的舵机；
        // 停止时每台都写一次，一台失联不妨碍其余舵机收到零输出或制动。
        result.acknowledged = true;
        bool failed = false;
        for (size_t i : chosen)
        {
            Ds485Servo *servo = servos_[i];
            bool sent = false;
            bool ok = false;
            if (addr == DS485_REG_MOTOR_PWM && width == 2)
            {
                ok = servo->WriteMotorPwm(GetI16(data[i].data()), &sent);
            }
            else
            {
                ok = servo->WriteReg(addr, data[i].data(), static_cast<int>(width), &sent);
            }
            if (sent)
            {
                result.sent = true;
                result.sent_ms = addr == DS485_REG_MOTOR_PWM ? servo->LastTargetSendMs() : servo->Port()->NowMs();
            }
            if (!ok)
            {
                result.acknowledged = false;
                if (!failed)
                {
                    result.error = servo->LastError();
                }
                failed = true;
                if (!best_effort)
                {
                    return result;
                }
                continue;
            }
            result.acknowledged = result.acknowledged && servo->WriteAckEnabled();
        }
        result.ok = !failed;
        return result;
    }

    SerialTransactionManager &bus = servos_[0]->Transactions();
    if (mode_ == SyncWrite::MultiId)
    {
        uint8_t ids[DS485_FRAME_MAX];
        uint8_t packed[DS485_FRAME_MAX];
        uint8_t payload[DS485_FRAME_MAX];
        int count = 0;
        for (size_t i : chosen)
        {
            if ((count + 1) * width > sizeof(packed))
            {
                result.error = "SYNC_FRAME_TOO_LONG";
                return result;
            }
            ids[count] = servos_[i]->Id();
            std::memcpy(packed + count * width, data[i].data(), width);
            ++count;
        }
        const int size = BuildMultiWritePayload(ids, count, packed, static_cast<int>(width), payload);
        if (!size)
        {
            result.error = "SYNC_FRAME_TOO_LONG";
            return result;
        }
        // 一帧同时送达总线上全部目标，舵机按协议不应答。
        Response r = bus.Exchange(DS485_ID_BROADCAST, DS485_CMD_MULTI_WRITE, addr, payload, size, 0, false);
        result.sent = r.sent;
        result.sent_ms = r.sent_ms;
        result.ok = r.ok;
        if (!r.ok)
        {
            result.error = Address("0x83 多 ID 写", addr) + r.error + (r.sent ? "，执行结果未知" : "");
        }
        return result;
    }

    // Staged：逐台放入缓存。0x84 之前任何其他指令都会清除缓存，因此中途失败时不广播执行帧。
    std::vector<size_t> staged;
    for (size_t i : chosen)
    {
        Response r = bus.Exchange(servos_[i]->Id(), DS485_CMD_STAGE, addr, data[i].data(),
                                  static_cast<int>(width), 0, false);
        if (r.sent)
        {
            staged.push_back(i);
        }
        if (!r.ok)
        {
            result.error = "舵机 ID " + std::to_string(servos_[i]->Id()) + "：" +
                           Address("0x04 同步写", addr) + r.error + "，未发送执行帧";
            ClearStaged(staged);
            return result;
        }
    }
    Response r = bus.Exchange(DS485_ID_BROADCAST, DS485_CMD_APPLY, 0, nullptr, 0, 0, false);
    result.sent = r.sent;
    result.sent_ms = r.sent_ms;
    result.ok = r.ok;
    if (!r.ok)
    {
        result.error = std::string("0x84 同步写执行 ") + r.error + (r.sent ? "，执行结果未知" : "");
        if (!r.sent)
        {
            ClearStaged(staged);
        }
    }
    return result;
}

SyncResult Ds485ServoGroup::WriteMotorPwm(const std::vector<int16_t> &pwm, const std::vector<bool> *mask,
                                          bool best_effort)
{
    std::vector<std::vector<uint8_t>> data;
    for (int16_t value : pwm)
    {
        if (value < -DS485_PWM_MAX || value > DS485_PWM_MAX)
        {
            SyncResult result;
            result.error = "PWM_OUT_OF_RANGE";
            return result;
        }
        std::vector<uint8_t> item(2);
        PutI16(item.data(), value);
        data.push_back(item);
    }
    return Write(DS485_REG_MOTOR_PWM, data, mask, best_effort);
}

SyncResult Ds485ServoGroup::WriteSwitch(uint8_t mode, const std::vector<bool> *mask, bool best_effort)
{
    if (mode > DS485_SWITCH_HOLD)
    {
        SyncResult result;
        result.error = "SWITCH_OUT_OF_RANGE";
        return result;
    }
    return Write(DS485_REG_SWITCH, std::vector<std::vector<uint8_t>>(servos_.size(), std::vector<uint8_t>{mode}),
                 mask, best_effort);
}
}  // namespace ds485
