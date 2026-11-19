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
    Response Exchange(uint8_t id, uint8_t cmd, uint8_t address, const uint8_t *data, int length,
        int expected, bool expects_ack);

  private:
    bool Interrupted() const;
    Rs485Port *port_;
    int gap_ms_ = 10, timeout_ms_ = 300, retries_ = 0;
    bool hex_ = false;
    // 只有总线工作线程维护。事务结果未知时设置隔离截止时刻，稳定排空后才允许下一笔普通事务。
    double quiet_until_ = 0;
    TransactionContext context_;
    FrameParser parser_;
    BusStatistics stats_;
    std::function<void(bool, const std::string &)> log_;
    std::function<bool()> debug_enabled_;
};
} // namespace ds485
