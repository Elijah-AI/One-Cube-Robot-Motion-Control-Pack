#ifndef DS485_DRIVER_MARVIN_RS485_PORT_H
#define DS485_DRIVER_MARVIN_RS485_PORT_H

/*
 * 第 1 层实现：经 Marvin 机械臂末端透传收发 485 数据（libMarvinSDK.so）。
 *
 * 数据流（详见 ds485/README.md “透传原理”一节）：
 *   发送：OnSetChDataA/B -> SDK 发送缓存 -> SDK 1ms 定时线程 -> UDP 发往 控制器IP:4729
 *         -> 控制器 -> 机械臂末端 COM1/COM2(485) -> 舵机
 *   接收：舵机应答 -> 末端 COM 口 -> 控制器 -> UDP 发往 本机:4730
 *         -> SDK 1ms 定时线程按臂放入接收队列 -> OnGetChDataA/B 每次取出一包
 *
 * 用法：
 *   MarvinLink::Connect("192.168.1.190", &error);   // 整个进程只连接一次
 *   MarvinRs485Port left('A', MARVIN_CH_COM1);       // A 臂末端 COM1
 *   left.Send(...); left.Receive(...);
 *   MarvinLink::Release();                           // 退出前必须释放
 *
 * 注意：
 *   - SDK 在本机绑定 UDP 4730 端口，同一台电脑同一时刻只能有一个进程连接机械臂。
 *     运控节点 motion_control_node 运行时，本节点无法连接（端口被占用），反之亦然。
 *   - SDK 内部是全局单例，Connect/Release 是全局操作，所以写成静态函数。
 */

#include "ds485_driver/rs485_port.h"

#include <cstdint>
#include <string>

// 末端通道号（SDK 中的 set_ch / ret_ch）
#define MARVIN_CH_CANFD 1   // CAN / CANFD
#define MARVIN_CH_COM1  2   // 末端 COM1（485）
#define MARVIN_CH_COM2  3   // 末端 COM2（485）

// 单包最大字节数（SDK 每包数据区 256 字节）
#define MARVIN_PACKET_MAX 256


/* 机械臂 SDK 连接管理（全局）。 */
class MarvinLink
{
public:
    /**
     * @brief 连接机械臂控制器
     * @param ip     控制器 IP，例如 "192.168.1.190"
     * @param error  失败原因（中文），可为空
     * @return true 成功
     *
     * 步骤：检查 SDK 结构体对齐 -> 关闭 SDK 本地打印 -> OnLinkTo -> 关闭日志
     *       -> 检查是否收到控制器状态包（只告警，不判失败，原因见 README）
     */
    static bool Connect(const std::string &ip, std::string *error);

    /**
     * @brief 本进程的其他模块已经调用过 OnLinkTo（例如把本驱动编进运控进程）时使用：
     *        只标记“已连接”，不再调用 OnLinkTo；之后 Release() 也不会调用 OnRelease，
     *        连接由原来的模块负责释放。
     */
    static void Attach();

    // 释放连接（关闭 UDP 端口与 SDK 定时线程）。未连接或 Attach 方式时不调用 OnRelease。
    static void Release();

    // 透传接口是否可用（Connect 成功或已 Attach）
    static bool IsConnected();

    /**
     * @brief 检查控制器状态包是否在持续到达（状态包序号在 wait_ms 内是否变化）
     * @return true：收到了新的状态包，说明 UDP 双向通路正常
     */
    static bool CheckAlive(int wait_ms);

private:
    static bool connected_;   // 透传接口可用
    static bool owner_;       // 连接是否由本类建立（是才负责 OnRelease）
};


/* 一条透传总线：某个臂 + 某个末端通道。 */
class MarvinRs485Port : public Rs485Port
{
public:
    /**
     * @param arm      'A' 或 'B'（A=左臂，B=右臂，与运控一致）
     * @param channel  MARVIN_CH_COM1 / MARVIN_CH_COM2 / MARVIN_CH_CANFD
     */
    MarvinRs485Port(char arm, int channel);
    virtual ~MarvinRs485Port() {}

    // Rs485Port 接口
    bool Send(const uint8_t *data, int size) override;
    bool SendBefore(const uint8_t *data, int size, double deadline) override;
    int Receive(uint8_t *buf, int max_size, int timeout_ms) override;
    void Clear() override;
    void SleepMs(int ms) override;
    std::string Name() const override;
    std::string LastError() const override;

    // 统计（调试用）
    long SentPackets() const { return sent_packets_; }
    long ReceivedPackets() const { return received_packets_; }
    long DroppedPackets() const { return dropped_packets_; }   // 其他通道的包，已丢弃

private:
    char arm_;
    int channel_;
    std::string last_error_;
    long sent_packets_;
    long received_packets_;
    long dropped_packets_;
};

#endif
