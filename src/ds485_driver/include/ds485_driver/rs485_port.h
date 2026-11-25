#ifndef DS485_DRIVER_RS485_PORT_H
#define DS485_DRIVER_RS485_PORT_H

/*
 * 第 1 层：通用 485 收发接口（与具体设备协议无关）。
 *
 * 上层（舵机协议层）只通过这个接口收发字节，不关心字节走的是哪条物理链路：
 *   - MarvinRs485Port ：经机械臂末端透传（libMarvinSDK.so，UDP -> 控制器 -> 末端 COM 口）
 *   - SerialRs485Port ：经 USB-485 串口（Linux 串口设备，台架调试用）
 *   - 离线测试中的模拟舵机也实现这个接口
 *
 * 约定：
 *   - 一个对象 = 一条总线（例如“A 臂末端 COM1”）。
 *   - 485 是半双工主从总线：调用者先 Send 一帧请求，再 Receive 读取应答。
 *   - Receive 返回的是“这一次收到的若干字节”，不保证是一帧完整报文，
 *     报文的拼接、切分、校验由协议层完成。
 *   - 本接口不加锁。多线程使用同一条总线时，由调用者保证同一时刻只有一个请求。
 */

#include <cstdint>
#include <chrono>
#include <string>

class Rs485Port
{
public:
    virtual ~Rs485Port() {}

    /**
     * @brief 发送 size 字节
     * @return true：已交给底层发送；false：发送失败，原因见 LastError()
     */
    virtual bool Send(const uint8_t *data, int size) = 0;
    // 截止时间使用同一端口的单调时钟。SDK 的完成时刻是进入发送槽的时刻。
    virtual bool SendBefore(const uint8_t *data, int size, double deadline)
    {
        return NowMs() < deadline && Send(data, size);
    }
    virtual double NowMs() const
    {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    /**
     * @brief 读取已收到的字节，最多等待 timeout_ms 毫秒
     * @param buf       接收缓冲区
     * @param max_size  缓冲区大小
     * @param timeout_ms 最长等待时间；0 表示只查看一次，不等待
     * @return >0：读到的字节数（读到一批就立即返回）
     *          0：等待超时，没有数据
     *         <0：读取失败，原因见 LastError()
     */
    virtual int Receive(uint8_t *buf, int max_size, int timeout_ms) = 0;

    // 丢弃接收缓存中残留的旧数据（发新请求前调用，避免读到上一次的应答）
    virtual void Clear() = 0;
    // 仅由总线线程在显式恢复时调用，普通轮询不会重开连接。
    virtual bool RecoverLink() { return true; }

    // 等待 ms 毫秒。实机为真实休眠；离线测试中用来推进模拟时间。
    virtual void SleepMs(int ms) = 0;

    // 总线名称，用于日志，例如 "marvin A/COM1"
    virtual std::string Name() const = 0;

    // 最近一次失败的原因（中文）
    virtual std::string LastError() const = 0;
};

#endif
