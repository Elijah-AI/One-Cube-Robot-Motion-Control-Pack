/*
 * 德晟舵机夹爪：MarvinSDK 末端透传收发实现。
 */
#include "Ds485_Marvin_Transport.h"

#include <string.h>
#include <unistd.h>

#include "MarvinSDK.h"


bool Ds485_Marvin_Transport::Clear(int side)
{
    if (side == DS485_SIDE_LEFT)
    {
        return OnClearChDataA();
    }
    if (side == DS485_SIDE_RIGHT)
    {
        return OnClearChDataB();
    }
    return false;
}

bool Ds485_Marvin_Transport::Send(int side, uint32_t channel, const uint8_t *data, int size)
{
    if (size <= 0 || size > 256)
    {
        return false;
    }
    // SDK 接口要求 256 字节可写缓冲区，这里复制一份
    unsigned char buf[256];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, size);

    if (side == DS485_SIDE_LEFT)
    {
        return OnSetChDataA(buf, size, (long)channel);
    }
    if (side == DS485_SIDE_RIGHT)
    {
        return OnSetChDataB(buf, size, (long)channel);
    }
    return false;
}

int Ds485_Marvin_Transport::Receive(int side, uint32_t channel, uint8_t *data, int max_size)
{
    unsigned char buf[256];
    memset(buf, 0, sizeof(buf));
    // 与 Modbus_Jodell.cpp 一致：传入请求的通道号，返回值为收到的字节数。
    // ret_ch 的返回值不作过滤（Jodell 实机代码也未过滤）。
    long ret_ch = (long)channel;
    long size = 0;

    if (side == DS485_SIDE_LEFT)
    {
        size = OnGetChDataA(buf, &ret_ch);
    }
    else if (side == DS485_SIDE_RIGHT)
    {
        size = OnGetChDataB(buf, &ret_ch);
    }
    else
    {
        return -1;
    }

    if (size <= 0)
    {
        return 0;   // 暂无数据
    }
    if (size > 256)
    {
        size = 256;
    }
    if (size > max_size)
    {
        size = max_size;
    }
    memcpy(data, buf, size);
    return (int)size;
}

void Ds485_Marvin_Transport::Sleep_ms(int ms)
{
    if (ms > 0)
    {
        usleep((useconds_t)ms * 1000);
    }
}
