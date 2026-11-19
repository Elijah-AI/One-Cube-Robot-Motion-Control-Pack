/*
 * 第 1 层实现：经 Marvin 机械臂末端透传收发 485 数据。
 * 只用到 libMarvinSDK.so 的以下接口：
 *   CheckSDKTypeCompat / OnLinkTo / OnRelease / OnLogOff / OnLocalLogOff / OnGetBuf
 *   OnClearChDataA/B / OnSetChDataA/B / OnGetChDataA/B
 */
#include "ds485_driver/marvin_rs485_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "MarvinSDK.h"


bool MarvinLink::connected_ = false;
bool MarvinLink::owner_ = false;

// 把 "192.168.1.190" 拆成 4 个数字，格式不对返回 false
static bool Parse_Ip(const std::string &ip, unsigned char out[4])
{
    int a = -1, b = -1, c = -1, d = -1;
    char tail = 0;
    if (sscanf(ip.c_str(), "%d.%d.%d.%d%c", &a, &b, &c, &d, &tail) != 4)
    {
        return false;
    }
    int parts[4] = {a, b, c, d};
    for (int i = 0; i < 4; i++)
    {
        if (parts[i] < 0 || parts[i] > 255)
        {
            return false;
        }
        out[i] = (unsigned char)parts[i];
    }
    return true;
}

static void Sleep_Ms(int ms)
{
    if (ms > 0)
    {
        usleep((useconds_t)ms * 1000);
    }
}


// ======================= MarvinLink =======================

bool MarvinLink::Connect(const std::string &ip, std::string *error)
{
    std::string dummy;
    if (error == 0)
    {
        error = &dummy;
    }
    if (connected_)
    {
        return true;
    }

    unsigned char ip_num[4];
    if (!Parse_Ip(ip, ip_num))
    {
        *error = "控制器 IP 格式错误：" + ip;
        return false;
    }

    // 1. 检查编译时结构体大小/对齐与 SDK 一致（与运控 checkCompatibility 相同）
    int byte_order = -1;
    int compat = CheckSDKTypeCompat(&byte_order);
    if (compat < 0 || (byte_order != 0 && byte_order != 1))
    {
        char text[120];
        snprintf(text, sizeof(text), "SDK 结构体检查失败（返回 %d），头文件与 libMarvinSDK.so 不匹配", compat);
        *error = text;
        return false;
    }

    // 2. 关闭 SDK 本地打印：默认每收到一包透传数据都会打印 256 字节十六进制，刷屏
    OnLocalLogOff();

    // 3. 连接：SDK 绑定本机 UDP 4730，目标为 控制器IP:4729，并启动 1ms 定时线程
    if (!OnLinkTo(ip_num[0], ip_num[1], ip_num[2], ip_num[3]))
    {
        *error = "OnLinkTo(" + ip + ") 失败。常见原因：本机 UDP 4730 端口已被其他程序占用"
                 "（例如 motion_control_node 正在运行），或本进程已连接过";
        return false;
    }
    connected_ = true;
    owner_ = true;
    OnLogOff();
    OnLocalLogOff();

    // 4. OnLinkTo 即使控制器没有回应也会返回 true，这里检查是否收到状态包，只告警
    if (!CheckAlive(500))
    {
        *error = "已调用 OnLinkTo，但 500ms 内未收到控制器状态包，请检查网线/IP/控制器是否上电";
        // 不返回 false：透传数据包与状态包是两条独立的数据，交给后续读写判断
    }
    else
    {
        error->clear();
    }
    return true;
}

void MarvinLink::Attach()
{
    // 连接由本进程其他模块建立：只关闭 SDK 本地打印，标记可用
    OnLocalLogOff();
    connected_ = true;
    owner_ = false;
}

void MarvinLink::Release()
{
    if (!connected_)
    {
        return;
    }
    if (owner_)
    {
        OnRelease();
    }
    connected_ = false;
    owner_ = false;
}

bool MarvinLink::IsConnected()
{
    return connected_;
}

bool MarvinLink::CheckAlive(int wait_ms)
{
    if (!connected_)
    {
        return false;
    }
    // DCSS 结构体较大，放在堆上
    DCSS *state = (DCSS *)calloc(1, sizeof(DCSS));
    if (state == 0)
    {
        return false;
    }
    bool alive = false;
    if (OnGetBuf(state))
    {
        long first = state->m_Out[0].m_OutFrameSerial;
        for (int waited = 0; waited < wait_ms; waited += 10)
        {
            Sleep_Ms(10);
            if (OnGetBuf(state) && state->m_Out[0].m_OutFrameSerial != first)
            {
                alive = true;
                break;
            }
        }
    }
    free(state);
    return alive;
}


// ======================= MarvinRs485Port =======================

MarvinRs485Port::MarvinRs485Port(char arm, int channel)
{
    arm_ = arm;
    channel_ = channel;
    sent_packets_ = 0;
    received_packets_ = 0;
    dropped_packets_ = 0;
}

bool MarvinRs485Port::Send(const uint8_t *data, int size)
{
    return SendBefore(data, size, NowMs() + 20);
}

bool MarvinRs485Port::SendBefore(const uint8_t *data, int size, double deadline)
{
    if (!MarvinLink::IsConnected())
    {
        last_error_ = "机械臂 SDK 未连接";
        return false;
    }
    if (arm_ != 'A' && arm_ != 'B')
    {
        last_error_ = "arm 参数只能是 A 或 B";
        return false;
    }
    if (data == 0 || size <= 0 || size > MARVIN_PACKET_MAX)
    {
        last_error_ = "发送长度必须是 1~256 字节";
        return false;
    }

    // SDK 接口参数是 256 字节可写数组，复制一份
    unsigned char buf[MARVIN_PACKET_MAX];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, size);

    // SDK 每个臂只有一个发送槽：上一包还没被 1ms 定时线程发出时返回 false。
    // 这里最多重试 20 次（约 20ms），仍失败才报错。
    for (int retry = 0; retry < 20 && NowMs() < deadline; retry++)
    {
        bool ok = false;
        if (arm_ == 'A')
        {
            ok = OnSetChDataA(buf, size, channel_);
        }
        else
        {
            ok = OnSetChDataB(buf, size, channel_);
        }
        if (ok)
        {
            sent_packets_++;
            return true;
        }
        Sleep_Ms(1);
    }
    last_error_ = "OnSetChData 连续 20ms 返回失败（发送槽一直被占用或通道号无效）";
    return false;
}

int MarvinRs485Port::Receive(uint8_t *buf, int max_size, int timeout_ms)
{
    if (!MarvinLink::IsConnected())
    {
        last_error_ = "机械臂 SDK 未连接";
        return -1;
    }
    if (buf == 0 || max_size <= 0)
    {
        last_error_ = "接收缓冲区无效";
        return -1;
    }

    int waited = 0;
    while (true)
    {
        unsigned char packet[MARVIN_PACKET_MAX];
        memset(packet, 0, sizeof(packet));
        // ret_ch 传入时必须是 1~3（SDK 会检查），返回时是这一包实际来自的通道
        long ret_ch = channel_;
        long size = 0;
        if (arm_ == 'A')
        {
            size = OnGetChDataA(packet, &ret_ch);
        }
        else
        {
            size = OnGetChDataB(packet, &ret_ch);
        }

        if (size > 0)
        {
            if (ret_ch != channel_)
            {
                // 同一个臂其他通道（例如 CANFD）的数据，不属于这条总线，丢弃
                dropped_packets_++;
                return 0;  // 让事务层重新检查截止时间
            }
            if (size > MARVIN_PACKET_MAX)
            {
                size = MARVIN_PACKET_MAX;
            }
            if (size > max_size)
            {
                size = max_size;   // 超出部分丢弃；协议层的缓冲区为 256 字节，不会发生
            }
            memcpy(buf, packet, size);
            received_packets_++;
            return (int)size;
        }

        if (waited >= timeout_ms)
        {
            return 0;
        }
        Sleep_Ms(1);
        waited++;
    }
}

void MarvinRs485Port::Clear()
{
    if (!MarvinLink::IsConnected())
    {
        return;
    }
    // 清空的是这个臂的整个接收队列（SDK 按臂排队，不分通道）
    if (arm_ == 'A')
    {
        OnClearChDataA();
    }
    else if (arm_ == 'B')
    {
        OnClearChDataB();
    }
}

void MarvinRs485Port::SleepMs(int ms)
{
    Sleep_Ms(ms);
}

std::string MarvinRs485Port::Name() const
{
    char text[40];
    const char *ch = "COM1";
    if (channel_ == MARVIN_CH_COM2)
    {
        ch = "COM2";
    }
    else if (channel_ == MARVIN_CH_CANFD)
    {
        ch = "CANFD";
    }
    snprintf(text, sizeof(text), "marvin %c/%s", arm_, ch);
    return text;
}

std::string MarvinRs485Port::LastError() const
{
    return last_error_;
}
