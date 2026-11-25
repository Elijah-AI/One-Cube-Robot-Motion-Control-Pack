#pragma once
#include "ds485_driver/ds485_protocol.h"
#include "ds485_driver/rs485_port.h"
#include <array>
#include <functional>
#include <string>

namespace ds485
{
struct TransactionContext
{
    std::string source = "initialization";
    double queued_ms = 0;
    double deadline_ms = 0;       // 该次工作的绝对截止时刻，0 表示使用单事务预算
    double response_limit_ms = 0; // 活动控制阶段按反馈周期收紧单次等待，0 沿用基础超时。
    bool stop = false;
    std::function<bool()> interrupted;
    double ack_window_ms = 0; // 定时 PWM 从实际发出起最多等候的毫秒数，0 使用普通应答窗。
};
struct BusStatistics
{
    uint64_t ok = 0, failed = 0, retries = 0, checksums = 0, timeouts = 0;
    unsigned consecutive = 0, max_consecutive = 0;
    std::array<double, 256> latency{};
    size_t samples = 0, next = 0;
    double maximum = 0;
    double Percentile(double fraction) const;
};
struct Response
{
    std::array<uint8_t, DS485_FRAME_MAX> frame{};
    bool ok = false, sent = false;
    int size = 0;
    std::string tx, rx, error;
    double sent_ms = -1;
};
class SerialTransactionManager
{
  public:
    explicit SerialTransactionManager(Rs485Port *port) : port_(port) {}
    void Timing(int gap, int timeout)
    {
        gap_ms_ = gap;
        timeout_ms_ = timeout;
    }
    void Options(int retries, bool hex)
    {
        retries_ = retries > 0 ? 1 : 0;
        hex_ = hex;
    }
    void Context(const TransactionContext &context)
    {
        context_ = context;
    }
    void Logger(
        std::function<void(bool, const std::string &)> log, std::function<bool()> debug_enabled = {})
    {
        log_ = std::move(log);
        debug_enabled_ = std::move(debug_enabled);
    }
    const BusStatistics &Stats() const
    {
        return stats_;
    }
    // 同一总线挂多台从站时置位。此时停止流程只允许广播帧或隔离来源的同一台抢占隔离，
    // 避免本次请求的应答与另一台的迟到包同时占线。
    void SharedBus(bool shared)
    {
        shared_bus_ = shared;
    }
    Response Exchange(uint8_t id, uint8_t cmd, uint8_t address, const uint8_t *data, int length,
        int expected, bool expects_ack);

    // 0x83/0x04/0x84 无应答帧之后的最短静默，留给各台处理同步帧；实际取 max(此值, gap_ms)。
    static constexpr int kNoReplySettleMs = 5;

  private:
    bool Interrupted() const;
    bool Preempts(uint8_t id) const;
    Rs485Port *port_;
    int gap_ms_ = 10, timeout_ms_ = 300, retries_ = 0;
    bool hex_ = false, shared_bus_ = false;
    // 只有总线工作线程维护。事务结果未知时设置隔离截止时刻，稳定排空后才允许下一笔普通事务。
    double quiet_until_ = 0;
    // 设置隔离的那笔事务的目标 ID；0xFF 表示来源不明（例如无应答帧后收到了字节）。
    uint8_t quiet_owner_ = 0xFF;
    // 当前隔离来自无应答帧。此期间收到字节说明有设备不按协议回包，记录一次警告。
    bool settling_ = false;
    TransactionContext context_;
    FrameParser parser_;
    BusStatistics stats_;
    std::function<void(bool, const std::string &)> log_;
    std::function<bool()> debug_enabled_;
};
} // namespace ds485
