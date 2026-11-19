/*
 * 模拟德晟舵机。寄存器含义与 ds485_protocol.h 一致。
 */
#include "fake_servo_port.h"

#include <string.h>

#include "ds485_driver/ds485_protocol.h"

using ds485::GetU16;
using ds485::PutU16;

static void Put_I32(uint8_t *p, int32_t value)
{
    uint32_t u = (uint32_t)value;
    p[0] = (uint8_t)(u & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)((u >> 16) & 0xFF);
    p[3] = (uint8_t)((u >> 24) & 0xFF);
}

FakeServoPort::FakeServoPort()
{
    time_ms = 0;
    Reset();
}

void FakeServoPort::Reset()
{
    memset(&servo, 0, sizeof(servo));
    servo.id = 1;
    servo.sys_config = 0x08;     // 写应答打开
    servo.min_limit = -1800;
    servo.max_limit = 1800;
    servo.mode = 1;              // 舵机模式
    servo.torque_switch = 0;     // 上电释放
    servo.pwm_limit = 1000;
    servo.current_limit = 3000;
    servo.speed_limit = 360;
    servo.temperature = 28;
    servo.voltage = 23941;
    servo.timed_read_size = 3;
    const uint8_t measured[4][4] = {{30, 5}, {60, 5}, {0x70, 0x17, 2}, {0x58, 2, 30}};
    memcpy(servo.thresholds, measured, sizeof(measured));
    tx_len_ = 0;
    split_pending_ = false;
}

uint8_t FakeServoPort::StateByte()
{
    uint8_t state = servo.fault_bits & 0x7F;
    if (servo.moving)
    {
        state |= 0x80;
    }
    return state;
}


// ======================= Rs485Port 接口 =======================

bool FakeServoPort::Send(const uint8_t *data, int size)
{
    if (servo.fail_send)
    {
        last_error_ = "模拟发送失败";
        return false;
    }
    if (size < 7 || data[0] != 0xF9 || data[1] != 0xFF || data[3] + 4 != size)
    {
        servo.frames_bad++;
        return true;
    }
    uint32_t sum = 0;
    for (int i = 2; i < size; i++)
    {
        sum += data[i];
    }
    if ((sum & 0xFF) != 0xFF)
    {
        servo.frames_bad++;
        return true;
    }
    servo.frames_ok++;
    if (data[2] == servo.id)
    {
        HandleFrame(data, size);
    }
    return true;
}

int FakeServoPort::Receive(uint8_t *buf, int max_size, int timeout_ms)
{
    if (tx_len_ <= 0)
    {
        SleepMs(timeout_ms);
        return 0;
    }
    int n = tx_len_;
    if (split_pending_ && n > 1)
    {
        n = n / 2;               // 分包：第一次只给一半
        split_pending_ = false;
    }
    if (n > max_size)
    {
        n = max_size;
    }
    memcpy(buf, tx_, n);
    memmove(tx_, tx_ + n, tx_len_ - n);
    tx_len_ -= n;
    return n;
}

void FakeServoPort::Clear()
{
    tx_len_ = 0;
    split_pending_ = false;
}

void FakeServoPort::SleepMs(int ms)
{
    for (int i = 0; i < ms; i++)
    {
        time_ms++;
        Step();
    }
}


// ======================= 运动模拟 =======================

void FakeServoPort::Step()
{
    FakeServo &s = servo;
    if (!s.moving)
    {
        return;
    }
    if (s.blocked)
    {
        s.blocked_ms++;
        if (s.stall_fault_ms > 0 && s.blocked_ms >= s.stall_fault_ms)
        {
            s.fault_bits |= 0x08;   // 堵转保护
        }
        return;
    }
    if (s.pwm_limit < 100)
    {
        return;   // 实机日志：PWM 太小时推不动
    }
    // °/s -> 每 ms 移动 speed/100 个 0.1°
    s.move_acc += s.move_speed;
    int step = s.move_acc / 100;
    s.move_acc %= 100;
    if (step == 0)
    {
        return;
    }
    int dir = s.target > s.angle ? 1 : -1;
    int32_t next = s.angle + dir * step;
    if ((dir > 0 && next >= s.target) || (dir < 0 && next <= s.target))
    {
        next = s.target;
    }
    if (s.obstacle_enabled)
    {
        if (dir > 0 && s.angle <= s.obstacle_angle && next >= s.obstacle_angle && s.target > s.obstacle_angle)
        {
            next = s.obstacle_angle;
            s.blocked = true;
        }
        if (dir < 0 && s.angle >= s.obstacle_angle && next <= s.obstacle_angle && s.target < s.obstacle_angle)
        {
            next = s.obstacle_angle;
            s.blocked = true;
        }
    }
    s.angle = next;
    if (!s.blocked && s.angle == s.target)
    {
        s.moving = false;
        s.move_acc = 0;
    }
}


// ======================= 指令处理 =======================

void FakeServoPort::HandleFrame(const uint8_t *frame, int size)
{
    FakeServo &s = servo;
    uint8_t cmd = frame[4];
    uint8_t addr = frame[5];
    const uint8_t *param = frame + 6;
    int len = size - 7;
    bool ack = (s.sys_config & 0x08) != 0;

    if (s.no_reply)
    {
        return;
    }
    if (cmd == DS485_CMD_READ)
    {
        s.read_count++;
        ReplyRead(addr, param, len);
        return;
    }
    if (cmd != DS485_CMD_WRITE)
    {
        ReplyShort(0x20 | StateByte());
        return;
    }

    s.write_count++;
    s.last_write_addr = addr;
    bool ok = true;
    uint8_t before = StateByte();
    if (addr == s.ignore_write_addr)
    {
        // 模拟固件接收但未应用。读回才能发现。
    }
    else if (addr == DS485_REG_RECOVERY && len == 1)
    {
        s.recovery = param[0];
    }
    else if (addr >= DS485_REG_VOLTAGE_TRIP && addr <= DS485_REG_STALL_TRIP)
    {
        int expected = 2;
        if (addr == DS485_REG_CURRENT_TRIP) expected = s.timed_read_size;
        if (addr == DS485_REG_STALL_TRIP) expected = 4;
        ok = len == expected;
        if (ok) memcpy(s.thresholds[addr - DS485_REG_VOLTAGE_TRIP], param, len);
    }
    else if (addr == DS485_REG_PWM_LIMIT && len == 2)
    {
        uint16_t v = GetU16(param);
        ok = v <= 1000;
        if (ok) s.pwm_limit = v;
    }
    else if (addr == DS485_REG_CURRENT_LIMIT && len == 2)
    {
        s.current_limit = GetU16(param);
    }
    else if (addr == DS485_REG_SPEED_LIMIT && len == 2)
    {
        s.speed_limit = GetU16(param);
    }
    else if (addr == DS485_REG_MODE && len == 1)
    {
        ok = param[0] <= 2;
        if (ok) s.mode = param[0];
    }
    else if (addr == DS485_REG_SWITCH && len == 1)
    {
        ok = !s.reject_switch_write && param[0] <= 3;
        if (ok)
        {
            // 任何开关写入都停止当前运动；2 = 在当前位置锁舵
            s.torque_switch = param[0];
            s.moving = false;
            s.blocked = false;
            s.blocked_ms = 0;
            s.target = s.angle;
            s.move_acc = 0;
        }
    }
    else if (addr == DS485_REG_MOVE_SPEED && len == 4)
    {
        ++s.motion_writes;
        s.fault_bits &= ~(s.resolved_causes & ~s.recovery & DS485_AUTO_FAULT_MASK);
        int16_t angle = (int16_t)GetU16(param);
        uint16_t speed = GetU16(param + 2);
        bool out_of_limit = (s.min_limit != 0 && angle < s.min_limit) ||
                            (s.max_limit != 0 && angle > s.max_limit);
        ok = s.torque_switch == 2 && speed > 0 && speed <= s.speed_limit && !out_of_limit && s.mode != 2;
        if (ok && !s.fault_bits)
        {
            s.target = angle;
            s.move_speed = speed;
            s.moving = s.target != s.angle;
            s.blocked = false;
            s.blocked_ms = 0;
            s.move_acc = 0;
        }
    }
    else
    {
        ok = false;
    }

    if (ack && addr != s.drop_write_ack_addr)
    {
        if (addr == DS485_REG_MOVE_SPEED && s.stale_motion_ack && ok)
        {
            ReplyShort(before);
            return;
        }
        ReplyShort(ok ? StateByte() : (uint8_t)(0x40 | StateByte()));
    }
}

int FakeServoPort::StatusReg(uint8_t addr, uint8_t *out)
{
    FakeServo &s = servo;
    if (addr == s.unreadable_addr) return 0;
    int dir = s.target > s.angle ? 1 : -1;
    int16_t speed = (s.moving && !s.blocked) ? (int16_t)(dir * s.move_speed) : 0;
    int16_t current = s.blocked ? 800 : (s.moving ? 300 : 50);
    int16_t pwm = s.blocked ? (int16_t)s.pwm_limit : (s.moving ? (int16_t)(s.pwm_limit / 2) : 0);
    switch (addr)
    {
    case DS485_REG_ANGLE:       Put_I32(out, s.angle); return 4;
    case DS485_REG_SPEED:       PutU16(out, (uint16_t)speed); return 2;
    case DS485_REG_CURRENT:     PutU16(out, (uint16_t)current); return 2;
    case DS485_REG_PWM:         PutU16(out, (uint16_t)pwm); return 2;
    case DS485_REG_TEMPERATURE: PutU16(out, (uint16_t)s.temperature); return 2;
    case DS485_REG_VOLTAGE:     PutU16(out, (uint16_t)s.voltage); return 2;
    case DS485_REG_ANGLE_ERROR: Put_I32(out, s.target - s.angle); return 4;
    default:                    return 0;
    }
}

void FakeServoPort::ReplyRead(uint8_t addr, const uint8_t *param, int param_len)
{
    FakeServo &s = servo;
    uint8_t data[64];
    int len = 0;

    if (addr == DS485_REG_MULTI_READ)
    {
        if (param_len <= 0)
        {
            ReplyShort(0x20 | StateByte());
            return;
        }
        for (int i = 0; i < param_len; i++)
        {
            int n = StatusReg(param[i], data + len);
            if (n == 0 || len + n > (int)sizeof(data))
            {
                ReplyShort(0x20 | StateByte());
                return;
            }
            len += n;
        }
        PushReply(DS485_CMD_READ, addr, data, len, false);
        return;
    }
    if (param_len != 0)
    {
        ReplyShort(0x20 | StateByte());
        return;
    }
    if (addr >= DS485_REG_VOLTAGE_TRIP && addr <= DS485_REG_STALL_TRIP)
    {
        len = addr < DS485_REG_CURRENT_TRIP ? 2 : s.timed_read_size;
        PushReply(DS485_CMD_READ, addr, s.thresholds[addr - DS485_REG_VOLTAGE_TRIP], len, false);
        return;
    }
    len = StatusReg(addr, data);
    if (len > 0)
    {
        PushReply(DS485_CMD_READ, addr, data, len, false);
        return;
    }
    switch (addr)
    {
    case DS485_REG_STATE:
        data[0] = StateByte();
        data[1] = (uint8_t)(time_ms & 0xFF);
        len = 2;
        break;
    case DS485_REG_SYS_CONFIG:    data[0] = s.sys_config; len = 1; break;
    case DS485_REG_USER_SAVE:     data[0] = s.save_mode; len = 1; break;
    case DS485_REG_RECOVERY:      data[0] = s.recovery; len = 1; break;
    case DS485_REG_MAX_PWM:       PutU16(data, 1000); len = 2; break;
    case DS485_REG_MIN_ANGLE:     PutU16(data, (uint16_t)s.min_limit); len = 2; break;
    case DS485_REG_MAX_ANGLE:     PutU16(data, (uint16_t)s.max_limit); len = 2; break;
    case DS485_REG_MODE:          data[0] = s.mode; len = 1; break;
    case DS485_REG_PWM_LIMIT:     PutU16(data, s.pwm_limit); len = 2; break;
    case DS485_REG_CURRENT_LIMIT: PutU16(data, s.current_limit); len = 2; break;
    case DS485_REG_SWITCH:        data[0] = s.torque_switch; len = 1; break;
    case DS485_REG_SPEED_LIMIT:
        if (s.hide_speed_limit)
        {
            ReplyShort(0x20 | StateByte());
            return;
        }
        PutU16(data, s.speed_limit);
        len = 2;
        break;
    default:
        ReplyShort(0x20 | StateByte());
        return;
    }
    PushReply(DS485_CMD_READ, addr, data, len, false);
}

void FakeServoPort::ReplyShort(uint8_t status)
{
    PushReply(0, 0, &status, 1, true);
}

void FakeServoPort::PushReply(uint8_t cmd, uint8_t addr, const uint8_t *data, int len, bool is_short)
{
    uint8_t frame[80];
    int size = 0;
    frame[0] = 0xF9;
    frame[1] = 0xF5;
    frame[2] = servo.id;
    if (is_short)
    {
        frame[3] = 2;
        frame[4] = data[0];
        size = 5;
    }
    else
    {
        frame[3] = (uint8_t)(3 + len);
        frame[4] = cmd;
        frame[5] = addr;
        memcpy(frame + 6, data, len);
        size = 6 + len;
    }
    uint32_t sum = 0;
    for (int i = 2; i < size; i++)
    {
        sum += frame[i];
    }
    frame[size++] = (uint8_t)((~sum) & 0xFF);

    if (servo.add_noise)
    {
        static const uint8_t noise[] = {0x55, 0xF9, 0x00, 0xF9, 0xF5, 0x01, 0xFF};
        PushBytes(noise, sizeof(noise));
    }
    if (servo.corrupt_next_reply)
    {
        uint8_t bad[80];
        memcpy(bad, frame, size);
        bad[size - 1] = (uint8_t)(bad[size - 1] + 1);
        PushBytes(bad, size);
        servo.corrupt_next_reply = false;
    }
    PushBytes(frame, size);
    if (servo.split_reply)
    {
        split_pending_ = true;
    }
}

void FakeServoPort::PushBytes(const uint8_t *data, int len)
{
    if (tx_len_ + len > FAKE_TX_SIZE)
    {
        return;
    }
    memcpy(tx_ + tx_len_, data, len);
    tx_len_ += len;
}
