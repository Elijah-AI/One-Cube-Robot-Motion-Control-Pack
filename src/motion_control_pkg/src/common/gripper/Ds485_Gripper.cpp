/*
 * 自制夹爪：德晟串口总线智能舵机（第五版协议）控制实现。
 * 协议细节与 ds485_control/ds485_servo.py（已实机验证）保持一致。
 * 本文件不依赖 MarvinSDK 与 ROS，收发通过 Ds485_Transport 完成。
 */
#include "Ds485_Gripper.h"

#include <stdio.h>
#include <string.h>


// ======================= 小端读写小工具 =======================

static uint16_t Get_U16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int16_t Get_I16(const uint8_t *p)
{
    return (int16_t)Get_U16(p);
}

static int32_t Get_I32(const uint8_t *p)
{
    uint32_t value = (uint32_t)p[0]
                   | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16)
                   | ((uint32_t)p[3] << 24);
    return (int32_t)value;
}

static void Put_U16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void Put_I16(uint8_t *p, int16_t value)
{
    Put_U16(p, (uint16_t)value);
}

static int32_t Abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static const char *Side_Name(int side)
{
    return side == DS485_SIDE_LEFT ? "左夹爪" : "右夹爪";
}


// ======================= 构造 / 配置 =======================

Ds485_Gripper::Ds485_Gripper(Ds485_Transport *transport)
{
    transport_ = transport;
    for (int side = 0; side < DS485_SIDE_NUM; side++)
    {
        // 默认值：保守的速度，角度未标定（open == close），必须由调用者配置。
        config_[side].channel = 2;          // COM1
        config_[side].id = 1;
        config_[side].open_angle = 0;
        config_[side].close_angle = 0;
        config_[side].max_speed = 90;       // speed=255 时 90°/s
        config_[side].default_speed = 30;   // speed=0 时 30°/s
        config_[side].tolerance = 10;       // 1.0°
        config_[side].still_band = 3;       // 0.3°
        config_[side].blocked_samples = 10;
        config_[side].gap_ms = 10;
        config_[side].timeout_ms = 300;

        initialized_[side] = false;
        write_ack_[side] = false;
        min_limit_[side] = 0;
        max_limit_[side] = 0;
        rx_len_[side] = 0;

        watch_target_[side] = 0;
        watch_anchor_angle_[side] = 0;
        watch_settled_[side] = 0;
        watch_still_[side] = 0;

        last_error_[side][0] = '\0';
    }
}

Ds485_Gripper::~Ds485_Gripper()
{
}

bool Ds485_Gripper::Check_Side(int side)
{
    return side == DS485_SIDE_LEFT || side == DS485_SIDE_RIGHT;
}

void Ds485_Gripper::Set_Error(int side, const char *text)
{
    snprintf(last_error_[side], DS485_ERROR_SIZE, "%s：%s", Side_Name(side), text);
}

const char *Ds485_Gripper::Ds485_Last_Error(int side) const
{
    if (side != DS485_SIDE_LEFT && side != DS485_SIDE_RIGHT)
    {
        return "side 参数错误";
    }
    return last_error_[side];
}

void Ds485_Gripper::Ds485_Config(int side, uint32_t channel, uint8_t id)
{
    if (!Check_Side(side))
    {
        return;
    }
    config_[side].channel = channel;
    config_[side].id = id;
    initialized_[side] = false;   // 换了通道或 ID，必须重新 Init
}

void Ds485_Gripper::Ds485_Set_Angle_Range(int side, int32_t open_angle, int32_t close_angle)
{
    if (!Check_Side(side))
    {
        return;
    }
    config_[side].open_angle = open_angle;
    config_[side].close_angle = close_angle;
    initialized_[side] = false;   // 需要重新检查舵机限位
}

void Ds485_Gripper::Ds485_Set_Speed(int side, uint16_t default_speed, uint16_t max_speed)
{
    if (!Check_Side(side))
    {
        return;
    }
    config_[side].default_speed = default_speed;
    config_[side].max_speed = max_speed;
}

void Ds485_Gripper::Ds485_Set_Detect(int side, int32_t tolerance, int32_t still_band, int blocked_samples)
{
    if (!Check_Side(side))
    {
        return;
    }
    config_[side].tolerance = tolerance;
    config_[side].still_band = still_band;
    config_[side].blocked_samples = blocked_samples;
}

void Ds485_Gripper::Ds485_Set_Bus_Timing(int side, int gap_ms, int timeout_ms)
{
    if (!Check_Side(side))
    {
        return;
    }
    config_[side].gap_ms = gap_ms;
    config_[side].timeout_ms = timeout_ms;
}

const Ds485_config &Ds485_Gripper::Ds485_Get_Config(int side) const
{
    if (side == DS485_SIDE_RIGHT)
    {
        return config_[DS485_SIDE_RIGHT];
    }
    return config_[DS485_SIDE_LEFT];
}


// ======================= 帧组装 / 解析 =======================

int Ds485_Gripper::Ds485_Build_Frame(uint8_t id, uint8_t cmd, uint8_t addr,
                                     const uint8_t *payload, int payload_len, uint8_t *out)
{
    // 整帧不能超过 256 字节：帧头2 + ID1 + 长度1 + 功能1 + 地址1 + 参数N + 校验1
    if (payload_len < 0 || payload_len > 249)
    {
        return 0;
    }
    out[0] = 0xF9;
    out[1] = 0xFF;
    out[2] = id;
    out[3] = (uint8_t)(3 + payload_len);
    out[4] = cmd;
    out[5] = addr;
    for (int i = 0; i < payload_len; i++)
    {
        out[6 + i] = payload[i];
    }
    // 校验从 ID 开始，包含长度、功能、地址和参数，不包含 F9 FF 帧头
    uint32_t sum = 0;
    for (int i = 2; i < 6 + payload_len; i++)
    {
        sum += out[i];
    }
    out[6 + payload_len] = (uint8_t)((~sum) & 0xFF);
    return 7 + payload_len;
}

int Ds485_Gripper::Pop_Reply(int side, uint8_t *frame)
{
    uint8_t *buf = rx_buf_[side];

    while (rx_len_[side] > 0)
    {
        // 1. 找应答帧头 F9 F5
        int start = -1;
        for (int i = 0; i + 1 < rx_len_[side]; i++)
        {
            if (buf[i] == 0xF9 && buf[i + 1] == 0xF5)
            {
                start = i;
                break;
            }
        }
        if (start < 0)
        {
            // 没有帧头：只保留可能是半个帧头的最后一个 F9
            if (buf[rx_len_[side] - 1] == 0xF9)
            {
                buf[0] = 0xF9;
                rx_len_[side] = 1;
            }
            else
            {
                rx_len_[side] = 0;
            }
            return 0;
        }

        // 2. 丢掉帧头前面的噪声
        if (start > 0)
        {
            memmove(buf, buf + start, rx_len_[side] - start);
            rx_len_[side] -= start;
        }

        // 3. 长度字段
        if (rx_len_[side] < 4)
        {
            return 0;   // 等待更多数据
        }
        int length = buf[3];
        if (length < 2 || length > 252)
        {
            // 长度不合理，丢一个字节继续找
            memmove(buf, buf + 1, rx_len_[side] - 1);
            rx_len_[side] -= 1;
            continue;
        }
        int total = length + 4;
        if (rx_len_[side] < total)
        {
            return 0;   // 帧还没收全
        }

        // 4. 校验：从 ID 到校验字节求和，低 8 位应为 0xFF
        uint32_t sum = 0;
        for (int i = 2; i < total; i++)
        {
            sum += buf[i];
        }
        if ((sum & 0xFF) != 0xFF)
        {
            // 坏校验，丢一个字节继续找后面的有效帧
            memmove(buf, buf + 1, rx_len_[side] - 1);
            rx_len_[side] -= 1;
            continue;
        }

        // 5. 有效帧，取出
        memcpy(frame, buf, total);
        memmove(buf, buf + total, rx_len_[side] - total);
        rx_len_[side] -= total;
        return total;
    }
    return 0;
}

bool Ds485_Gripper::Send_Frame(int side, uint8_t cmd, uint8_t addr, const uint8_t *payload, int len)
{
    if (transport_ == 0)
    {
        Set_Error(side, "未设置收发接口 transport");
        return false;
    }

    uint8_t frame[256];
    int size = Ds485_Build_Frame(config_[side].id, cmd, addr, payload, len, frame);
    if (size <= 0)
    {
        Set_Error(side, "指令参数过长");
        return false;
    }

    // 主从半双工总线：每次请求串行执行。先留出间隔，再清掉上一请求的残留数据。
    transport_->Sleep_ms(config_[side].gap_ms);
    transport_->Clear(side);
    rx_len_[side] = 0;

    if (!transport_->Send(side, config_[side].channel, frame, size))
    {
        char text[100];
        snprintf(text, sizeof(text), "透传发送失败（地址 0x%02X）", addr);
        Set_Error(side, text);
        return false;
    }
    return true;
}


// ======================= 寄存器读写 =======================

bool Ds485_Gripper::Ds485_Read_Reg(int side, uint8_t addr, uint8_t *data, int expect_len)
{
    if (!Check_Side(side))
    {
        return false;
    }
    if (!Send_Frame(side, DS485_CMD_READ, addr, 0, 0))
    {
        return false;
    }

    int polls = config_[side].timeout_ms / 2;
    if (polls < 1)
    {
        polls = 1;
    }

    uint8_t chunk[256];
    uint8_t frame[256];
    for (int i = 0; i < polls; i++)
    {
        int got = transport_->Receive(side, config_[side].channel, chunk, sizeof(chunk));
        if (got < 0)
        {
            Set_Error(side, "透传读取失败");
            return false;
        }
        if (got > 0)
        {
            if (rx_len_[side] + got > DS485_RX_BUF_SIZE)
            {
                rx_len_[side] = 0;   // 缓存溢出，丢弃旧数据
            }
            memcpy(rx_buf_[side] + rx_len_[side], chunk, got);
            rx_len_[side] += got;
        }

        int size = Pop_Reply(side, frame);
        while (size > 0)
        {
            if (frame[2] == config_[side].id)
            {
                if (frame[3] == 2)
                {
                    // 读命令收到短应答：说明舵机拒绝了本次读取
                    char faults[120];
                    char text[180];
                    Ds485_Fault_Text(frame[4], faults, sizeof(faults));
                    snprintf(text, sizeof(text), "读取 0x%02X 收到短应答 0x%02X（%s）",
                             addr, frame[4], faults);
                    Set_Error(side, text);
                    return false;
                }
                if (frame[4] == DS485_CMD_READ && frame[5] == addr)
                {
                    int data_len = frame[3] - 3;
                    if (data_len != expect_len)
                    {
                        char text[120];
                        snprintf(text, sizeof(text), "0x%02X 应答长度错误：预期 %d，实际 %d",
                                 addr, expect_len, data_len);
                        Set_Error(side, text);
                        return false;
                    }
                    memcpy(data, frame + 6, data_len);
                    return true;
                }
            }
            size = Pop_Reply(side, frame);   // 不是本次应答，继续找
        }

        if (got == 0)
        {
            transport_->Sleep_ms(2);
        }
    }

    char text[120];
    snprintf(text, sizeof(text), "ID %d 读取 0x%02X 超时，未收到有效应答", config_[side].id, addr);
    Set_Error(side, text);
    return false;
}

bool Ds485_Gripper::Ds485_Write_Reg(int side, uint8_t addr, const uint8_t *payload, int len)
{
    if (!Check_Side(side))
    {
        return false;
    }
    if (!Send_Frame(side, DS485_CMD_WRITE, addr, payload, len))
    {
        return false;
    }

    if (!write_ack_[side])
    {
        // 舵机未开启写应答：不等待不存在的短应答，由调用者读回确认
        transport_->Sleep_ms(20);
        return true;
    }

    int polls = config_[side].timeout_ms / 2;
    if (polls < 1)
    {
        polls = 1;
    }

    uint8_t chunk[256];
    uint8_t frame[256];
    for (int i = 0; i < polls; i++)
    {
        int got = transport_->Receive(side, config_[side].channel, chunk, sizeof(chunk));
        if (got < 0)
        {
            Set_Error(side, "透传读取失败");
            return false;
        }
        if (got > 0)
        {
            if (rx_len_[side] + got > DS485_RX_BUF_SIZE)
            {
                rx_len_[side] = 0;
            }
            memcpy(rx_buf_[side] + rx_len_[side], chunk, got);
            rx_len_[side] += got;
        }

        int size = Pop_Reply(side, frame);
        while (size > 0)
        {
            if (frame[2] == config_[side].id && frame[3] == 2)
            {
                uint8_t status = frame[4];
                if (status & DS485_STATE_FAULT_MASK)
                {
                    char faults[120];
                    char text[180];
                    Ds485_Fault_Text(status, faults, sizeof(faults));
                    snprintf(text, sizeof(text), "写入 0x%02X 失败，短应答 0x%02X（%s）",
                             addr, status, faults);
                    Set_Error(side, text);
                    return false;
                }
                return true;
            }
            size = Pop_Reply(side, frame);
        }

        if (got == 0)
        {
            transport_->Sleep_ms(2);
        }
    }

    // 写指令不自动重发，避免重复执行
    char text[120];
    snprintf(text, sizeof(text), "写入 0x%02X 应答超时，执行结果未知（不会重发）", addr);
    Set_Error(side, text);
    return false;
}

bool Ds485_Gripper::Ds485_Read_U8(int side, uint8_t addr, uint8_t *value)
{
    uint8_t data[1];
    if (!Ds485_Read_Reg(side, addr, data, 1))
    {
        return false;
    }
    *value = data[0];
    return true;
}

bool Ds485_Gripper::Ds485_Read_U16(int side, uint8_t addr, uint16_t *value)
{
    uint8_t data[2];
    if (!Ds485_Read_Reg(side, addr, data, 2))
    {
        return false;
    }
    *value = Get_U16(data);
    return true;
}

bool Ds485_Gripper::Ds485_Read_I16(int side, uint8_t addr, int16_t *value)
{
    uint8_t data[2];
    if (!Ds485_Read_Reg(side, addr, data, 2))
    {
        return false;
    }
    *value = Get_I16(data);
    return true;
}

bool Ds485_Gripper::Ds485_Read_I32(int side, uint8_t addr, int32_t *value)
{
    uint8_t data[4];
    if (!Ds485_Read_Reg(side, addr, data, 4))
    {
        return false;
    }
    *value = Get_I32(data);
    return true;
}

bool Ds485_Gripper::Ds485_Write_U8_Check(int side, uint8_t addr, uint8_t value)
{
    uint8_t data[1];
    data[0] = value;
    if (!Ds485_Write_Reg(side, addr, data, 1))
    {
        return false;
    }
    uint8_t actual = 0;
    if (!Ds485_Read_U8(side, addr, &actual))
    {
        return false;
    }
    if (actual != value)
    {
        char text[120];
        snprintf(text, sizeof(text), "0x%02X 读回不一致：设置 %d，实际 %d", addr, value, actual);
        Set_Error(side, text);
        return false;
    }
    return true;
}

bool Ds485_Gripper::Ds485_Write_U16_Check(int side, uint8_t addr, uint16_t value)
{
    uint8_t data[2];
    Put_U16(data, value);
    if (!Ds485_Write_Reg(side, addr, data, 2))
    {
        return false;
    }
    uint16_t actual = 0;
    if (!Ds485_Read_U16(side, addr, &actual))
    {
        return false;
    }
    if (actual != value)
    {
        char text[120];
        snprintf(text, sizeof(text), "0x%02X 读回不一致：设置 %d，实际 %d", addr, value, actual);
        Set_Error(side, text);
        return false;
    }
    return true;
}


// ======================= 换算 =======================

int32_t Ds485_Gripper::Ds485_Position_To_Angle(int side, uint8_t position) const
{
    const Ds485_config &c = Ds485_Get_Config(side);
    int64_t diff = (int64_t)c.close_angle - (int64_t)c.open_angle;
    return (int32_t)(c.open_angle + (diff * position) / 255);
}

uint8_t Ds485_Gripper::Ds485_Angle_To_Position(int side, int32_t angle) const
{
    const Ds485_config &c = Ds485_Get_Config(side);
    int64_t den = (int64_t)c.close_angle - (int64_t)c.open_angle;
    if (den == 0)
    {
        return 0;
    }
    int64_t num = ((int64_t)angle - (int64_t)c.open_angle) * 255;
    if (den < 0)
    {
        num = -num;
        den = -den;
    }
    // 四舍五入
    int64_t pos = 0;
    if (num >= 0)
    {
        pos = (num + den / 2) / den;
    }
    else
    {
        pos = -((-num + den / 2) / den);
    }
    if (pos < 0)
    {
        pos = 0;
    }
    if (pos > 255)
    {
        pos = 255;
    }
    return (uint8_t)pos;
}

void Ds485_Gripper::Ds485_Fault_Text(uint8_t state, char *text, int text_size)
{
    // 下标即状态字节中的位号
    static const char *names[7] = {
        "电压异常保护",        // Bit0
        "过流保护",            // Bit1
        "过温保护",            // Bit2
        "堵转/过载保护",       // Bit3
        "硬件异常保护",        // Bit4
        "校验异常或命令未开放", // Bit5
        "命令执行失败",        // Bit6
    };
    if (text == 0 || text_size <= 0)
    {
        return;
    }
    text[0] = '\0';
    int used = 0;
    for (int bit = 0; bit < 7; bit++)
    {
        if (state & (1 << bit))
        {
            int n = snprintf(text + used, text_size - used, "%s%s",
                             used > 0 ? "、" : "", names[bit]);
            if (n < 0 || used + n >= text_size)
            {
                break;
            }
            used += n;
        }
    }
    if (used == 0)
    {
        snprintf(text, text_size, "无故障");
    }
}


// ======================= 设备操作 =======================

bool Ds485_Gripper::Ds485_Init(int side)
{
    if (!Check_Side(side))
    {
        return false;
    }
    initialized_[side] = false;
    Ds485_config &c = config_[side];

    if (c.open_angle == c.close_angle)
    {
        Set_Error(side, "张开/闭合角度未标定（open == close），请先配置 ds485_gripper_open_deg / close_deg");
        return false;
    }
    if (c.open_angle < -32768 || c.open_angle > 32767 || c.close_angle < -32768 || c.close_angle > 32767)
    {
        Set_Error(side, "张开/闭合角度超出协议范围 -3276.8°~3276.7°");
        return false;
    }
    if (c.max_speed == 0 || c.default_speed == 0)
    {
        Set_Error(side, "速度配置不能为 0");
        return false;
    }

    // 1. 写应答设置：0x11 Bit3=1 时写指令有短应答
    uint8_t sys_config = 0;
    if (!Ds485_Read_U8(side, DS485_REG_SYS_CONFIG, &sys_config))
    {
        return false;
    }
    write_ack_[side] = (sys_config & 0x08) != 0;

    // 2. 舵机角度限位，0 表示该侧不限制
    if (!Ds485_Read_I16(side, DS485_REG_MIN_ANGLE, &min_limit_[side]))
    {
        return false;
    }
    if (!Ds485_Read_I16(side, DS485_REG_MAX_ANGLE, &max_limit_[side]))
    {
        return false;
    }

    // 3. 控制模式：只支持自动(0)/舵机(1)。电机模式(2)需人工切回，代码不自动切换。
    uint8_t mode = 0;
    if (!Ds485_Read_U8(side, DS485_REG_MODE, &mode))
    {
        return false;
    }
    if (mode != 0 && mode != 1)
    {
        char text[120];
        snprintf(text, sizeof(text), "当前控制模式为 %d，不是自动/舵机模式，请先切回舵机模式", mode);
        Set_Error(side, text);
        return false;
    }

    // 4. 标定角度必须在舵机限位内
    initialized_[side] = true;
    if (!Check_Move_Target(side, c.open_angle) || !Check_Move_Target(side, c.close_angle))
    {
        initialized_[side] = false;
        return false;
    }
    return true;
}

bool Ds485_Gripper::Check_Move_Target(int side, int32_t angle)
{
    const Ds485_config &c = config_[side];
    int32_t low = c.open_angle < c.close_angle ? c.open_angle : c.close_angle;
    int32_t high = c.open_angle < c.close_angle ? c.close_angle : c.open_angle;
    char text[160];

    if (angle < low || angle > high)
    {
        snprintf(text, sizeof(text), "目标 %.1f° 超出标定范围 %.1f°~%.1f°",
                 angle / 10.0, low / 10.0, high / 10.0);
        Set_Error(side, text);
        return false;
    }
    if (min_limit_[side] != 0 && angle < min_limit_[side])
    {
        snprintf(text, sizeof(text), "目标 %.1f° 低于舵机限位 %.1f°",
                 angle / 10.0, min_limit_[side] / 10.0);
        Set_Error(side, text);
        return false;
    }
    if (max_limit_[side] != 0 && angle > max_limit_[side])
    {
        snprintf(text, sizeof(text), "目标 %.1f° 高于舵机限位 %.1f°",
                 angle / 10.0, max_limit_[side] / 10.0);
        Set_Error(side, text);
        return false;
    }
    return true;
}

bool Ds485_Gripper::Ds485_Move_Angle(int side, int32_t angle, uint16_t speed)
{
    if (!Check_Side(side))
    {
        return false;
    }
    if (!initialized_[side])
    {
        Set_Error(side, "未初始化，请先调用 Ds485_Init");
        return false;
    }
    if (speed == 0)
    {
        Set_Error(side, "速度不能为 0");
        return false;
    }
    if (!Check_Move_Target(side, angle))
    {
        return false;
    }

    // 1. 舵机有保护异常时不下发，异常需人工查明，代码不自动清除
    uint8_t state[2];
    if (!Ds485_Read_Reg(side, DS485_REG_STATE, state, 2))
    {
        return false;
    }
    if (state[0] & DS485_STATE_FAULT_MASK)
    {
        char faults[120];
        char text[180];
        Ds485_Fault_Text(state[0], faults, sizeof(faults));
        snprintf(text, sizeof(text), "舵机状态异常 0x%02X（%s），未下发运动", state[0], faults);
        Set_Error(side, text);
        return false;
    }

    // 2. 扭矩开关不是“锁舵”时先打开输出
    uint8_t torque_switch = 0;
    if (!Ds485_Read_U8(side, DS485_REG_SWITCH, &torque_switch))
    {
        return false;
    }
    if (torque_switch != DS485_SWITCH_HOLD)
    {
        if (!Ds485_Write_U8_Check(side, DS485_REG_SWITCH, DS485_SWITCH_HOLD))
        {
            return false;
        }
    }

    // 3. 定速绝对角度 0x66：[角度 Int16, 0.1°][速度 Uint16, °/s]
    uint8_t payload[4];
    Put_I16(payload, (int16_t)angle);
    Put_U16(payload + 2, speed);
    if (!Ds485_Write_Reg(side, DS485_REG_MOVE_SPEED, payload, 4))
    {
        return false;
    }
    watch_target_[side] = angle;
    return true;
}

bool Ds485_Gripper::Ds485_Move_Abs(int side, uint8_t position, uint8_t torque, uint8_t speed)
{
    if (!Check_Side(side))
    {
        return false;
    }
    const Ds485_config &c = config_[side];

    // 力矩 0~255 -> PWM 上限 0~1000；0 表示沿用舵机当前 PWM 上限
    if (torque > 0)
    {
        uint16_t pwm = (uint16_t)((torque * 1000 + 127) / 255);
        if (!Ds485_Set_Torque(side, pwm))
        {
            return false;
        }
    }

    // 速度 0~255 -> °/s；0 表示使用默认速度
    uint16_t speed_dps = c.default_speed;
    if (speed > 0)
    {
        speed_dps = (uint16_t)((speed * (uint32_t)c.max_speed + 127) / 255);
        if (speed_dps < 1)
        {
            speed_dps = 1;
        }
    }

    int32_t angle = Ds485_Position_To_Angle(side, position);
    return Ds485_Move_Angle(side, angle, speed_dps);
}

bool Ds485_Gripper::Ds485_Set_Torque(int side, uint16_t pwm)
{
    if (!Check_Side(side))
    {
        return false;
    }
    if (pwm > 1000)
    {
        Set_Error(side, "PWM 上限范围为 0~1000");
        return false;
    }
    return Ds485_Write_U16_Check(side, DS485_REG_PWM_LIMIT, pwm);
}

bool Ds485_Gripper::Ds485_Set_Current_Limit(int side, uint16_t current_ma)
{
    if (!Check_Side(side))
    {
        return false;
    }
    return Ds485_Write_U16_Check(side, DS485_REG_CURRENT_LIMIT, current_ma);
}

bool Ds485_Gripper::Ds485_Set_Speed_Limit(int side, uint16_t speed_dps)
{
    if (!Check_Side(side))
    {
        return false;
    }
    if (speed_dps == 0)
    {
        Set_Error(side, "速度上限不能为 0");
        return false;
    }
    return Ds485_Write_U16_Check(side, DS485_REG_SPEED_LIMIT, speed_dps);
}

bool Ds485_Gripper::Ds485_Status(int side, Ds485_status *rt_status)
{
    if (!Check_Side(side) || rt_status == 0)
    {
        return false;
    }
    uint8_t state[2];
    if (!Ds485_Read_Reg(side, DS485_REG_STATE, state, 2))
    {
        return false;
    }
    uint8_t s = state[0];
    rt_status->state_raw = s;
    rt_status->err_voltage      = (s & 0x01) != 0;
    rt_status->err_over_current = (s & 0x02) != 0;
    rt_status->err_over_temp    = (s & 0x04) != 0;
    rt_status->err_stall        = (s & 0x08) != 0;
    rt_status->err_hardware     = (s & 0x10) != 0;
    rt_status->err_checksum     = (s & 0x20) != 0;
    rt_status->err_cmd_failed   = (s & 0x40) != 0;
    rt_status->moving           = (s & 0x80) != 0;

    if (!Ds485_Read_I32(side, DS485_REG_ANGLE, &rt_status->angle))
    {
        return false;
    }
    if (!Ds485_Read_I16(side, DS485_REG_SPEED, &rt_status->speed))
    {
        return false;
    }
    if (!Ds485_Read_I16(side, DS485_REG_CURRENT, &rt_status->current))
    {
        return false;
    }
    if (!Ds485_Read_I16(side, DS485_REG_PWM, &rt_status->pwm))
    {
        return false;
    }
    rt_status->position = Ds485_Angle_To_Position(side, rt_status->angle);
    return true;
}

bool Ds485_Gripper::Ds485_Read_Health(int side, Ds485_status *rt_status)
{
    if (!Check_Side(side) || rt_status == 0)
    {
        return false;
    }
    if (!Ds485_Read_I16(side, DS485_REG_TEMPERATURE, &rt_status->temperature))
    {
        return false;
    }
    if (!Ds485_Read_I16(side, DS485_REG_VOLTAGE, &rt_status->voltage))
    {
        return false;
    }
    if (!Ds485_Read_U8(side, DS485_REG_MODE, &rt_status->mode))
    {
        return false;
    }
    if (!Ds485_Read_U8(side, DS485_REG_SWITCH, &rt_status->torque_switch))
    {
        return false;
    }
    if (!Ds485_Read_U16(side, DS485_REG_PWM_LIMIT, &rt_status->pwm_limit))
    {
        return false;
    }
    return true;
}

bool Ds485_Gripper::Ds485_Hold(int side)
{
    if (!Check_Side(side))
    {
        return false;
    }
    return Ds485_Write_U8_Check(side, DS485_REG_SWITCH, DS485_SWITCH_HOLD);
}

bool Ds485_Gripper::Ds485_Brake(int side)
{
    if (!Check_Side(side))
    {
        return false;
    }
    return Ds485_Write_U8_Check(side, DS485_REG_SWITCH, DS485_SWITCH_BRAKE);
}

bool Ds485_Gripper::Ds485_Release(int side)
{
    if (!Check_Side(side))
    {
        return false;
    }
    return Ds485_Write_U8_Check(side, DS485_REG_SWITCH, DS485_SWITCH_RELEASE);
}


// ======================= 到位 / 夹物判断 =======================

int32_t Ds485_Gripper::Ds485_Target_Angle(int side) const
{
    if (side == DS485_SIDE_RIGHT)
    {
        return watch_target_[DS485_SIDE_RIGHT];
    }
    return watch_target_[DS485_SIDE_LEFT];
}

void Ds485_Gripper::Ds485_Watch_Start(int side, int32_t target, int32_t start)
{
    if (!Check_Side(side))
    {
        return;
    }
    watch_target_[side] = target;
    watch_anchor_angle_[side] = start;
    watch_settled_[side] = 0;
    watch_still_[side] = 0;
}

int Ds485_Gripper::Ds485_Watch_Update(int side, const Ds485_status &rt_status)
{
    if (!Check_Side(side))
    {
        return DS485_MOVE_FAULT;
    }
    const Ds485_config &c = config_[side];

    // 1. 任一保护位 -> 故障
    if (rt_status.state_raw & DS485_STATE_FAULT_MASK)
    {
        char faults[120];
        char text[180];
        Ds485_Fault_Text(rt_status.state_raw, faults, sizeof(faults));
        snprintf(text, sizeof(text), "运动中舵机状态异常 0x%02X（%s）", rt_status.state_raw, faults);
        Set_Error(side, text);
        return DS485_MOVE_FAULT;
    }

    int32_t error = Abs32(rt_status.angle - watch_target_[side]);

    // 2. 到位：转动位清零且在容差内，连续两次确认
    if (!rt_status.moving && error <= c.tolerance)
    {
        watch_settled_[side]++;
    }
    else
    {
        watch_settled_[side] = 0;
    }
    if (watch_settled_[side] >= 2)
    {
        return DS485_MOVE_ARRIVED;
    }

    // 3. 角度连续多次几乎不变：在容差内算到位，否则判定夹到物体
    //    与“基准角度”比较而不是与上一次比较，慢速运动时位移会累积超出 still_band，不会误判
    if (Abs32(rt_status.angle - watch_anchor_angle_[side]) <= c.still_band)
    {
        watch_still_[side]++;
    }
    else
    {
        watch_anchor_angle_[side] = rt_status.angle;   // 还在动，重新取基准
        watch_still_[side] = 0;
    }

    if (watch_still_[side] >= c.blocked_samples)
    {
        if (error <= c.tolerance)
        {
            return DS485_MOVE_ARRIVED;
        }
        return DS485_MOVE_BLOCKED;
    }
    return DS485_MOVE_RUNNING;
}
