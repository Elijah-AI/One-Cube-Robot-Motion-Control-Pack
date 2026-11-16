#ifndef __DS485_MARVIN_TRANSPORT_H
#define __DS485_MARVIN_TRANSPORT_H

/*
 * 德晟舵机夹爪的实机收发：通过 MarvinSDK 机械臂末端透传接口。
 *   左夹爪 -> 机械臂 A：OnClearChDataA / OnSetChDataA / OnGetChDataA
 *   右夹爪 -> 机械臂 B：OnClearChDataB / OnSetChDataB / OnGetChDataB
 * 透传方式与 Modbus_Jodell.cpp 相同。使用前机器人 SDK 必须已连接（OnLinkTo）。
 */

#include "Ds485_Gripper.h"

class Ds485_Marvin_Transport : public Ds485_Transport
{
public:
    Ds485_Marvin_Transport() {}
    virtual ~Ds485_Marvin_Transport() {}

    virtual bool Clear(int side);
    virtual bool Send(int side, uint32_t channel, const uint8_t *data, int size);
    virtual int Receive(int side, uint32_t channel, uint8_t *data, int max_size);
    virtual void Sleep_ms(int ms);
};

#endif
