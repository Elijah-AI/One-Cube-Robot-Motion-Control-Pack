/*
 * 模拟德晟舵机实现。寄存器含义与 Ds485_Gripper.h 一致。
 */
#include "fake_ds485_servo.h"

#include <string.h>


static uint16_t Get_U16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static void Put_U16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void Put_I32(uint8_t *p, int32_t value)
{
    uint32_t u = (uint32_t)value;
    p[0] = (uint8_t)(u & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)((u >> 16) & 0xFF);
    p[3] = (uint8_t)((u >> 24) & 0xFF);
}


Fake_Ds485_Bus::Fake_Ds485_Bus()
{
    time_ms = 0;
    for (int side = 0; side < DS485_SIDE_NUM; side++)
    {
        Reset_Servo(side);
    }
}

void Fake_Ds485_Bus::Reset_Servo(int side)
{
    Fake_Servo &s = servo[side];
    memset(&s, 0, sizeof(s));
    s.id = 1;
    s.sys_config = 0x08;        // 默认打开写应答
    s.min_limit = -1800;        // -180.0°
    s.max_limit = 1800;         // 180.0°
    s.mode = 1;                 // 舵机模式
    s.torque_switch = 0;        // 上电释放
    s.pwm_limit = 1000;
    s.current_limit = 3000;
    s.speed_limit = 360;
    s.temperature = 28;
    s.voltage = 23941;
    s.angle = 0;
    s.target = 0;

    tx_len_[side] = 0;
    split_left_[side] = 0;
}

uint8_t Fake_Ds485_Bus::State_Byte(int side)
{
    uint8_t state = servo[side].fault_bits & 0x7F;
    if (servo[side].moving)
    {
        state |= 0x80;
    }
    return state;
}


// ======================= 透传接口 =======================

bool Fake_Ds485_Bus::Clear(int side)
{
    tx_len_[side] = 0;
    split_left_[side] = 0;
    return true;
}

bool Fake_Ds485_Bus::Send(int side, uint32_t channel, const uint8_t *data, int size)
{
    Fake_Servo &s = servo[side];
    s.last_channel = channel;

    // 帧格式：F9 FF ID LEN CMD ADDR 参数 CHK，LEN = 3 + 参数长度
    if (size < 7 || data[0] != 0xF9 || data[1] != 0xFF || data[3] + 4 != size)
    {
        s.frames_bad++;
        return true;   // 串口已发出，但舵机不认这一帧
    }
    uint32_t sum = 0;
    for (int i = 2; i < size; i++)
    {
        sum += data[i];
    }
    if ((sum & 0xFF) != 0xFF)
    {
        s.frames_bad++;
        return true;
    }
    s.frames_ok++;
    if (data[2] != s.id)
    {
        return true;   // 不是发给这个舵机的
    }
    Handle_Frame(side, data, size);
    return true;
}

int Fake_Ds485_Bus::Receive(int side, uint32_t channel, uint8_t *data, int max_size)
{
    (void)channel;
    int n = tx_len_[side];
    if (n <= 0)
    {
        return 0;
    }
    // 分包模式：第一次只给一半
    if (split_left_[side] == 1 && n > 1)
    {
        n = n / 2;
        split_left_[side] = 0;
    }
    if (n > max_size)
    {
        n = max_size;
    }
    memcpy(data, tx_[side], n);
    memmove(tx_[side], tx_[side] + n, tx_len_[side] - n);
    tx_len_[side] -= n;
    return n;
}

void Fake_Ds485_Bus::Sleep_ms(int ms)
{
    if (ms <= 0)
    {
        return;
    }
    // 按 1ms 步长推进，便于判断“经过物体”的时刻
    for (int i = 0; i < ms; i++)
    {
        time_ms++;
        for (int side = 0; side < DS485_SIDE_NUM; side++)
        {
            Step(side, 1);
        }
    }
}


// ======================= 运动模拟 =======================

void Fake_Ds485_Bus::Step(int side, int ms)
{
    Fake_Servo &s = servo[side];
    if (!s.moving)
    {
        return;
    }
    if (s.blocked)
    {
        // 被物体挡住：持续用力，超过设定时间触发堵转保护
        s.blocked_ms += ms;
        if (s.stall_fault_ms > 0 && s.blocked_ms >= s.stall_fault_ms)
        {
            s.fault_bits |= 0x08;
        }
        return;
    }
    if (s.pwm_limit == 0)
    {
        return;   // 没有驱动力，转不动
    }

    // 速度单位 °/s = 10 个 0.1° / 1000ms，即每 ms 移动 speed/100 个单位
    s.move_acc += s.move_speed * ms;
    int step = s.move_acc / 100;
    s.move_acc = s.move_acc % 100;
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

    // 物体挡在运动路径上（目标在物体之后）
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

void Fake_Ds485_Bus::Handle_Frame(int side, const uint8_t *frame, int size)
{
    Fake_Servo &s = servo[side];
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
        if (addr == 0x59)
        {
            Reply_Combined(side, param, len);
            return;
        }
        Reply_Read(side, addr);
        return;
    }
    if (cmd != DS485_CMD_WRITE)
    {
        Reply_Short(side, 0x20 | State_Byte(side));   // 命令未开放
        return;
    }

    s.write_count++;
    s.last_write_addr = addr;
    bool ok = true;

    if (addr == DS485_REG_PWM_LIMIT && len == 2)
    {
        uint16_t value = Get_U16(param);
        if (value > 1000)
        {
            ok = false;
        }
        else
        {
            s.pwm_limit = value;
        }
    }
    else if (addr == DS485_REG_CURRENT_LIMIT && len == 2)
    {
        s.current_limit = Get_U16(param);
    }
    else if (addr == DS485_REG_SPEED_LIMIT && len == 2)
    {
        s.speed_limit = Get_U16(param);
    }
    else if (addr == DS485_REG_MODE && len == 1)
    {
        if (param[0] > 2)
        {
            ok = false;
        }
        else
        {
            s.mode = param[0];
        }
    }
    else if (addr == DS485_REG_SWITCH && len == 1)
    {
        if (s.reject_switch_write || param[0] > 3)
        {
            ok = false;
        }
        else
        {
            s.torque_switch = param[0];
            // 任何开关写入都会停止当前运动；2=在当前位置锁舵
            s.moving = false;
            s.blocked = false;
            s.blocked_ms = 0;
            s.target = s.angle;
            s.move_acc = 0;
        }
    }
    else if (addr == DS485_REG_MOVE_SPEED && len == 4)
    {
        int16_t angle = (int16_t)Get_U16(param);
        uint16_t speed = Get_U16(param + 2);
        bool out_of_limit = (s.min_limit != 0 && angle < s.min_limit) ||
                            (s.max_limit != 0 && angle > s.max_limit);
        if (s.torque_switch != 2 || speed == 0 || out_of_limit || s.mode == 2)
        {
            ok = false;
        }
        else
        {
            s.target = angle;
            s.move_speed = speed;
            s.moving = (s.target != s.angle);
            s.blocked = false;
            s.blocked_ms = 0;
            s.move_acc = 0;
        }
    }
    else
    {
        ok = false;
    }

    if (!ack)
    {
        return;   // 写应答关闭：成功或失败都不回复
    }
    if (ok)
    {
        Reply_Short(side, State_Byte(side));
    }
    else
    {
        Reply_Short(side, 0x40 | State_Byte(side));
    }
}

void Fake_Ds485_Bus::Reply_Combined(int side, const uint8_t *addresses, int len)
{
    const uint8_t expected[] = {0x46, 0x47, 0x48, 0x49};
    if (len != 4 || memcmp(addresses, expected, sizeof(expected)) != 0)
    {
        Reply_Short(side, 0x40 | State_Byte(side));
        return;
    }
    const Fake_Servo &s = servo[side];
    const int direction = s.target > s.angle ? 1 : -1;
    const int16_t speed = (s.moving && !s.blocked) ? direction * s.move_speed : 0;
    const int16_t current = s.blocked ? 800 : (s.moving ? 300 : 50);
    const int16_t pwm = s.blocked ? s.pwm_limit : (s.moving ? s.pwm_limit / 2 : 0);
    uint8_t data[10];
    Put_I32(data, s.angle);
    Put_U16(data + 4, static_cast<uint16_t>(speed));
    Put_U16(data + 6, static_cast<uint16_t>(current));
    Put_U16(data + 8, static_cast<uint16_t>(pwm));
    Push_Reply(side, DS485_CMD_READ, 0x59, data, sizeof(data), false);
}

void Fake_Ds485_Bus::Reply_Read(int side, uint8_t addr)
{
    Fake_Servo &s = servo[side];
    uint8_t data[8];
    int len = 0;
    int dir = s.target > s.angle ? 1 : -1;
    int16_t speed = (s.moving && !s.blocked) ? (int16_t)(dir * s.move_speed) : 0;
    int16_t current = s.blocked ? 800 : (s.moving ? 300 : 50);
    int16_t pwm = s.blocked ? (int16_t)s.pwm_limit : (s.moving ? (int16_t)(s.pwm_limit / 2) : 0);

    switch (addr)
    {
    case DS485_REG_STATE:
        data[0] = State_Byte(side);
        data[1] = (uint8_t)(time_ms & 0xFF);   // 随机数
        len = 2;
        break;
    case DS485_REG_SYS_CONFIG:
        data[0] = s.sys_config;
        len = 1;
        break;
    case DS485_REG_MAX_PWM:
        Put_U16(data, 1000);
        len = 2;
        break;
    case DS485_REG_MIN_ANGLE:
        Put_U16(data, (uint16_t)s.min_limit);
        len = 2;
        break;
    case DS485_REG_MAX_ANGLE:
        Put_U16(data, (uint16_t)s.max_limit);
        len = 2;
        break;
    case DS485_REG_ANGLE:
        Put_I32(data, s.angle);
        len = 4;
        break;
    case DS485_REG_SPEED:
        Put_U16(data, (uint16_t)speed);
        len = 2;
        break;
    case DS485_REG_CURRENT:
        Put_U16(data, (uint16_t)current);
        len = 2;
        break;
    case DS485_REG_PWM:
        Put_U16(data, (uint16_t)pwm);
        len = 2;
        break;
    case DS485_REG_TEMPERATURE:
        Put_U16(data, (uint16_t)s.temperature);
        len = 2;
        break;
    case DS485_REG_VOLTAGE:
        Put_U16(data, (uint16_t)s.voltage);
        len = 2;
        break;
    case DS485_REG_MODE:
        data[0] = s.mode;
        len = 1;
        break;
    case DS485_REG_PWM_LIMIT:
        Put_U16(data, s.pwm_limit);
        len = 2;
        break;
    case DS485_REG_CURRENT_LIMIT:
        Put_U16(data, s.current_limit);
        len = 2;
        break;
    case DS485_REG_SPEED_LIMIT:
        Put_U16(data, s.speed_limit);
        len = 2;
        break;
    case DS485_REG_SWITCH:
        data[0] = s.torque_switch;
        len = 1;
        break;
    default:
        Reply_Short(side, 0x20 | State_Byte(side));   // 地址未开放
        return;
    }
    Push_Reply(side, DS485_CMD_READ, addr, data, len, false);
}

void Fake_Ds485_Bus::Reply_Short(int side, uint8_t status)
{
    Push_Reply(side, 0, 0, &status, 1, true);
}

void Fake_Ds485_Bus::Push_Reply(int side, uint8_t cmd, uint8_t addr, const uint8_t *data, int len, bool is_short)
{
    Fake_Servo &s = servo[side];
    uint8_t frame[64];
    int size = 0;

    frame[0] = 0xF9;
    frame[1] = 0xF5;
    frame[2] = s.id;
    if (is_short)
    {
        // F9 F5 ID 02 STATUS CHK
        frame[3] = 2;
        frame[4] = data[0];
        size = 5;
    }
    else
    {
        // F9 F5 ID LEN CMD ADDR 数据 CHK
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
    frame[size] = (uint8_t)((~sum) & 0xFF);
    size++;

    if (s.add_noise)
    {
        // 噪声：无关字节 + 一个长度非法的假帧头
        static const uint8_t noise[] = {0x55, 0xF9, 0x00, 0xF9, 0xF5, 0x01, 0xFF};
        Push_Bytes(side, noise, sizeof(noise));
    }
    if (s.corrupt_next_reply)
    {
        // 先插入一帧校验错误的应答，紧接着才是正确应答
        uint8_t bad[64];
        memcpy(bad, frame, size);
        bad[size - 1] = (uint8_t)(bad[size - 1] + 1);
        Push_Bytes(side, bad, size);
        s.corrupt_next_reply = false;
    }
    Push_Bytes(side, frame, size);
    if (s.split_reply)
    {
        split_left_[side] = 1;
    }
}

void Fake_Ds485_Bus::Push_Bytes(int side, const uint8_t *data, int len)
{
    if (tx_len_[side] + len > FAKE_TX_SIZE)
    {
        return;
    }
    memcpy(tx_[side] + tx_len_[side], data, len);
    tx_len_[side] += len;
}
