/*
 * 第 2 层（协议部分）：报文组装与解析。
 * 与 ds485_control/ds485_servo.py、motion_control_pkg/Ds485_Gripper.cpp（均已实机验证）一致。
 */
#include "ds485_driver/ds485_protocol.h"

#include <stdio.h>
#include <string.h>

namespace ds485
{

uint16_t GetU16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

int16_t GetI16(const uint8_t *p)
{
    return (int16_t)GetU16(p);
}

int32_t GetI32(const uint8_t *p)
{
    uint32_t value = (uint32_t)p[0]
                   | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16)
                   | ((uint32_t)p[3] << 24);
    return (int32_t)value;
}

void PutU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
}

void PutI16(uint8_t *p, int16_t value)
{
    PutU16(p, (uint16_t)value);
}

int BuildFrame(uint8_t id, uint8_t cmd, uint8_t addr,
               const uint8_t *payload, int payload_len, uint8_t *out)
{
    // 帧头2 + ID1 + 长度1 + 功能1 + 地址1 + 参数N + 校验1，总长不超过 256
    if (!out || (payload_len>0 && !payload) || payload_len < 0 || payload_len > DS485_FRAME_MAX - 7)
    {
        return 0;
    }
    out[0] = 0xF9;
    out[1] = 0xFF;
    out[2] = id;
    if (cmd==DS485_CMD_QUERY || cmd==DS485_CMD_APPLY) {
        if (payload_len) return 0;
        out[3]=2;
        out[4]=cmd;
        out[5]=static_cast<uint8_t>(~(id+2+cmd));
        return 6;
    }
    out[3] = (uint8_t)(3 + payload_len);
    out[4] = cmd;
    out[5] = addr;
    for (int i = 0; i < payload_len; i++)
    {
        out[6 + i] = payload[i];
    }
    // 校验从 ID 开始（不含 F9 FF 帧头）
    uint32_t sum = 0;
    for (int i = 2; i < 6 + payload_len; i++)
    {
        sum += out[i];
    }
    out[6 + payload_len] = (uint8_t)((~sum) & 0xFF);
    return 7 + payload_len;
}

std::string FaultText(uint8_t state)
{
    // 下标即状态字节中的位号
    static const char *names[7] = {
        "电压异常保护",         // Bit0
        "过流保护",             // Bit1
        "过温保护",             // Bit2
        "堵转/过载保护",        // Bit3
        "硬件异常保护",         // Bit4
        "校验异常或命令未开放", // Bit5
        "命令执行失败",         // Bit6
    };
    std::string text;
    for (int bit = 0; bit < 7; bit++)
    {
        if (state & (1 << bit))
        {
            if (!text.empty())
            {
                text += "、";
            }
            text += names[bit];
        }
    }
    if (text.empty())
    {
        text = "无故障";
    }
    return text;
}

std::string ToHex(const uint8_t *data, int size)
{
    std::string text;
    char item[4];
    for (int i = 0; i < size; i++)
    {
        snprintf(item, sizeof(item), i == 0 ? "%02X" : " %02X", data[i]);
        text += item;
    }
    return text;
}


// ======================= FrameParser =======================

FrameParser::FrameParser()
{
    len_ = 0;
    dropped_ = 0;
}

void FrameParser::Reset()
{
    len_ = 0;
    bad_header = bad_length = bad_checksum = 0;
}

void FrameParser::Push(const uint8_t *data, int size)
{
    if (size <= 0)
    {
        return;
    }
    if (size > (int)sizeof(buf_))
    {
        // 一次来的数据比缓存还大：只保留最后一段
        data += size - (int)sizeof(buf_);
        size = (int)sizeof(buf_);
    }
    if (len_ + size > (int)sizeof(buf_))
    {
        Drop(len_ + size - (int)sizeof(buf_));   // 丢弃最旧的字节
    }
    memcpy(buf_ + len_, data, size);
    len_ += size;
}

void FrameParser::Drop(int count)
{
    if (count >= len_)
    {
        dropped_ += len_;
        len_ = 0;
        return;
    }
    memmove(buf_, buf_ + count, len_ - count);
    len_ -= count;
    dropped_ += count;
}

int FrameParser::Pop(uint8_t *frame)
{
    while (len_ > 0)
    {
        // 1. 找应答帧头 F9 F5
        int start = -1;
        for (int i = 0; i + 1 < len_; i++)
        {
            if (buf_[i] == 0xF9 && buf_[i + 1] == 0xF5)
            {
                start = i;
                break;
            }
        }
        if (start < 0)
        {
            if (len_ > 1 || buf_[0] != 0xF9) ++bad_header;
            // 没有帧头：只保留最后一个可能是半个帧头的 F9
            if (buf_[len_ - 1] == 0xF9)
            {
                Drop(len_ - 1);
            }
            else
            {
                Drop(len_);
            }
            return 0;
        }

        // 2. 丢掉帧头前的噪声
        if (start > 0)
        {
            ++bad_header;
            Drop(start);
        }

        // 3. 长度字段
        if (len_ < 4)
        {
            return 0;   // 等待更多数据
        }
        int length = buf_[3];
        if (length < 2 || length > DS485_FRAME_MAX - 4)
        {
            ++bad_length;
            Drop(1);    // 长度不合理，丢一个字节继续找
            continue;
        }
        int total = length + 4;
        if (len_ < total)
        {
            return 0;   // 帧还没收全
        }

        // 4. 校验：从 ID 到校验字节求和，低 8 位应为 0xFF
        uint32_t sum = 0;
        for (int i = 2; i < total; i++)
        {
            sum += buf_[i];
        }
        if ((sum & 0xFF) != 0xFF)
        {
            ++bad_checksum;
            Drop(1);    // 坏校验，丢一个字节继续找后面的有效帧
            continue;
        }

        // 5. 有效帧
        memcpy(frame, buf_, total);
        memmove(buf_, buf_ + total, len_ - total);
        len_ -= total;
        return total;
    }
    return 0;
}

}  // namespace ds485
