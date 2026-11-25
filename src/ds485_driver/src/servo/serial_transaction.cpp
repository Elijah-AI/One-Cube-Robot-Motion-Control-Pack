#include "ds485_driver/serial_transaction.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <sstream>

namespace ds485
{
namespace
{
int RegisterWidth(uint8_t addr)
{
    switch (addr)
    {
    case 0x04:
    case 0x0f:
    case 0x11:
    case 0x32:
    case 0x33:
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x5a:
    case 0x64:
        return 1;
    case 0x01:
    case 0x10:
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
    case 0x19:
    case 0x1a:
    case 0x1c:
    case 0x3c:
    case 0x3d:
    case 0x47:
    case 0x48:
    case 0x49:
    case 0x4a:
    case 0x4b:
    case 0x5b:
    case 0x5c:
    case 0x5d:
        return 2;
    case 0x0b:
    case 0x0c:
    case 0x46:
    case 0x4c:
    case 0x4d:
        return 4;
    case 0x0d:
        return 8;
    default:
        return 0;
    }
}
} // namespace
double BusStatistics::Percentile(double fraction) const
{
    if (!samples)
    {
        return 0;
    }
    auto values = latency;
    const size_t index =
        static_cast<size_t>(std::ceil(std::clamp(fraction, 1. / samples, 1.) * samples)) - 1;
    std::nth_element(values.begin(), values.begin() + index, values.begin() + samples);
    return values[index];
}

bool SerialTransactionManager::Interrupted() const
{
    return context_.interrupted && context_.interrupted();
}

bool SerialTransactionManager::Preempts(uint8_t id) const
{
    // 紧急停止可以抢占隔离。单台总线沿用原规则。多台总线上，另一台的迟到包可能仍在线上，
    // 发给其他舵机的请求会让两台同时回包，因此只放行广播停止帧和隔离来源的同一台。
    return context_.stop && (!shared_bus_ || id == DS485_ID_BROADCAST || id == quiet_owner_);
}

Response SerialTransactionManager::Exchange(uint8_t id, uint8_t cmd, uint8_t address, const uint8_t *data,
    int length, int expected, bool expects_ack)
{
    static std::atomic<uint64_t> sequence{0};
    Response result;
    uint8_t tx[DS485_FRAME_MAX];
    int size = BuildFrame(id, cmd, address, data, length, tx);
    const bool read = cmd == DS485_CMD_READ;
    const bool query = cmd == DS485_CMD_QUERY;
    const bool unicast = id <= 250;
    const bool ordinary_write = cmd == DS485_CMD_WRITE && (unicast || id == 254);
    const bool staged =
        (cmd == DS485_CMD_STAGE && unicast) || (cmd == DS485_CMD_APPLY && (unicast || id == 254));
    const bool group_write = cmd == DS485_CMD_MULTI_WRITE && id == 254;
    if (!port_ || !size || (!((read || query) && unicast) && !ordinary_write && !staged && !group_write))
    {
        result.error = "INVALID_REQUEST";
        return result;
    }
    const uint64_t txid = ++sequence;
    expects_ack = (read || query) || (ordinary_write && unicast && expects_ack);
    if (read)
    {
        int width = RegisterWidth(address);
        if (address == DS485_REG_MULTI_READ)
        {
            if (!length)
            {
                result.error = "INVALID_REGISTER_LIST";
                return result;
            }
            for (int i = 0; i < length; ++i)
            {
                if (data[i] < 0x46 || data[i] > 0x4d)
                {
                    result.error = "INVALID_REGISTER_LIST";
                    return result;
                }
                width += RegisterWidth(data[i]);
            }
        }
        if (width > DS485_FRAME_MAX - 7 || (width && expected > 0 && expected != width))
        {
            result.error = "RX_BAD_LENGTH";
            return result;
        }
        if (width)
        {
            expected = width;
        }
    }
    const int attempts = read ? retries_ + 1 : 1;
    const double entered = port_->NowMs();
    const double response_limit = context_.response_limit_ms > 0
                                      ? std::min(double(timeout_ms_), context_.response_limit_ms)
                                      : timeout_ms_;
    // 隔离、发送、所有重试共用上层预算。错帧和分包均不能延长任何一次应答截止时间。
    const double budget = context_.deadline_ms > 0
                              ? context_.deadline_ms
                              : std::max(entered, quiet_until_) + attempts * (gap_ms_ + timeout_ms_ + 2) +
                                    (attempts - 1) * (timeout_ms_ + gap_ms_ + 2);
    result.tx = ToHex(tx, size);
    for (int attempt = 1; attempt <= attempts; ++attempt)
    {
        if (attempt > 1)
        {
            ++stats_.retries;
        }
        result.sent = false;
        result.size = 0;
        result.rx.clear();
        std::string reason, unmatched_reason;
        bool timeout = false;
        double started = port_->NowMs(), first = -1, written = -1, send_start = -1;
        int count = 0, captured = 0, mismatches = 0;
        uint8_t raw[2 * DS485_FRAME_MAX];
        // 上一次超时后，即使没有新字节，也等过一个应答窗。遇到迟到包继续等待静默间隔。
        // 紧急停止可以抢占隔离（多台总线的限制见 Preempts）；其执行结果另由异类型读请求确认。
        uint8_t chunk[DS485_FRAME_MAX];
        while (!Preempts(id) && port_->NowMs() < quiet_until_ && port_->NowMs() < budget && !Interrupted())
        {
            int n = port_->Receive(chunk, sizeof(chunk), 0);
            if (n < 0)
            {
                reason = "SERIAL_READ_ERROR";
                break;
            }
            if (n > 0)
            {
                quiet_until_ = std::max(quiet_until_, port_->NowMs() + std::max(2, gap_ms_));
                if (settling_)
                {
                    // 无应答帧之后不该有字节。来源不明，之后任何一台都不能抢占这段隔离。
                    settling_ = false;
                    quiet_owner_ = 0xFF;
                    if (log_)
                    {
                        std::ostringstream line;
                        line << "UNSOLICITED_RX device=" << port_->Name() << " bytes=" << n
                             << " after_no_reply_frame next_servo_id=" << int(id) << " rx_hex=[" << ToHex(chunk, n)
                             << "]";
                        log_(true, line.str());
                    }
                }
            }
            port_->SleepMs(1);
        }
        double gap_end = port_->NowMs() + gap_ms_;
        while (port_->NowMs() < gap_end && port_->NowMs() < budget && !Interrupted())
        {
            port_->SleepMs(1);
        }
        if (Interrupted())
        {
            reason = "TX_CANCELED";
        }
        else if (port_->NowMs() >= budget)
        {
            reason = "RX_TIMEOUT";
        }
        parser_.Reset();
        if (reason.empty())
        {
            settling_ = false;
            port_->Clear();
            send_start = port_->NowMs();
            const double send_deadline = std::min(budget, send_start + response_limit);
            result.sent = port_->SendBefore(tx, size, send_deadline);
            written = port_->NowMs();
            if (!result.sent)
            {
                reason = "SERIAL_WRITE_FAILED";
            }
            else
            {
                result.sent_ms = written;
            }
        }
        double deadline = written >= 0 ? std::min(budget, written + response_limit) : budget;
        if (!read && written >= 0 && context_.ack_window_ms > 0)
        {
            deadline = std::min(deadline, written + context_.ack_window_ms);
        }
        if (result.sent && written >= budget)
        {
            reason = "TX_DEADLINE_EXCEEDED";
        }
        if (result.sent && written < budget && !read && !expects_ack)
        {
            result.ok = true;
            reason = "TX_NO_ACK_EXPECTED";
            const double settle = written + std::max(kNoReplySettleMs, gap_ms_);
            if ((group_write || staged) && settle > quiet_until_)
            {
                // 同步帧没有应答，也无法知道各台何时处理完。下一帧前至少静默一段，
                // 期间收到的字节按迟到包处理并延长隔离。单台普通写保持原时序。
                quiet_until_ = settle;
                quiet_owner_ = id;
                settling_ = true;
            }
        }
        while (result.sent && !result.ok && port_->NowMs() < deadline && !Interrupted())
        {
            int n = port_->Receive(chunk, sizeof(chunk), 0);
            if (n < 0)
            {
                reason = "SERIAL_READ_ERROR";
                break;
            }
            if (n > 0)
            {
                if (first < 0)
                {
                    first = port_->NowMs();
                }
                count += n;
                int keep = std::min(n, static_cast<int>(sizeof(raw)) - captured);
                std::copy_n(chunk, keep, raw + captured);
                captured += keep;
                parser_.Push(chunk, n);
            }
            int bytes;
            while ((bytes = parser_.Pop(result.frame.data())) > 0)
            {
                const auto &f = result.frame;
                if (f[2] != id)
                {
                    reason = "RX_ID_MISMATCH";
                }
                else if (read && f[3] == 2 && !(f[4] & DS485_STATE_FAULT_MASK))
                {
                    reason = "RX_STALE_OR_UNMATCHED";
                }
                else if (!read && f[3] != 2)
                {
                    reason = "RX_STALE_OR_UNMATCHED";
                }
                else if (read && f[3] != 2 && f[4] != cmd)
                {
                    reason = "RX_COMMAND_MISMATCH";
                }
                else if (read && f[3] != 2 && f[5] != address)
                {
                    reason = "RX_ADDRESS_MISMATCH";
                }
                else if (read && f[3] != 2 && (address == 0x3e || address == 0x3f) && bytes - 7 != 3 &&
                         bytes - 7 != 4)
                {
                    reason = "RX_BAD_LENGTH";
                }
                else if (read && f[3] != 2 && expected > 0 && bytes - 7 != expected)
                {
                    reason = "RX_BAD_LENGTH";
                }
                else
                {
                    result.size = bytes;
                    result.rx = ToHex(f.data(), bytes);
                    result.ok = true;
                    reason.clear();
                    break;
                }
                unmatched_reason = reason;
                ++mismatches;
            }
            if (n == 0 && !result.ok)
            {
                port_->SleepMs(1);
            }
        }
        if (result.ok && result.sent && expects_ack && port_->NowMs() >= deadline)
        {
            result.ok = false;
            reason = "RX_TIMEOUT";
        }
        const double end = port_->NowMs();
        if (!result.ok && result.sent)
        {
            timeout = end >= deadline;
            if (Interrupted())
            {
                reason = "TX_CANCELED";
            }
            else if (reason.empty())
            {
                if (parser_.bad_checksum)
                {
                    reason = "RX_CHECKSUM_ERROR";
                }
                else if (parser_.bad_length)
                {
                    reason = "RX_BAD_LENGTH";
                }
                else if (parser_.Pending())
                {
                    reason = "RX_PARTIAL_FRAME";
                }
                else if (parser_.bad_header)
                {
                    reason = "RX_BAD_HEADER";
                }
                else
                {
                    reason = count ? "RX_STALE_OR_UNMATCHED" : "RX_NO_BYTES";
                }
            }
            quiet_until_ = end + timeout_ms_;
            quiet_owner_ = id;
            settling_ = false;
        }
        if (!result.sent && reason == "SERIAL_WRITE_FAILED")
        {
            quiet_until_ = end + timeout_ms_;
            quiet_owner_ = id;
            settling_ = false;
        }
        if (reason == "RX_TIMEOUT")
        {
            timeout = true;
        }
        stats_.checksums += parser_.bad_checksum;
        if (timeout)
        {
            ++stats_.timeouts;
        }
        const bool negative =
            result.ok && result.size == 6 && (read || (result.frame[4] & DS485_STATE_FAULT_MASK));
        const bool failed = !result.ok || negative;
        if (failed)
        {
            ++stats_.failed;
            ++stats_.consecutive;
            stats_.max_consecutive = std::max(stats_.max_consecutive, stats_.consecutive);
        }
        else
        {
            ++stats_.ok;
            stats_.consecutive = 0;
            if (first >= 0)
            {
                const double latency = end - written;
                stats_.latency[stats_.next] = latency;
                stats_.next = (stats_.next + 1) % stats_.latency.size();
                stats_.samples = std::min(stats_.samples + 1, stats_.latency.size());
                stats_.maximum = std::max(stats_.maximum, latency);
            }
        }
        const bool retry = !result.ok && read && attempt < attempts && !Interrupted() &&
                           budget > std::max(end, quiet_until_) + gap_ms_ + 1;
        const bool warn = failed || mismatches > 0 || parser_.bad_checksum > 0;
        if (log_ && (warn || !debug_enabled_ || debug_enabled_()))
        {
            std::ostringstream line;
            line.precision(2);
            line << std::fixed << "txid=" << txid << " device=" << port_->Name() << " servo_id=" << int(id)
                 << " cmd=0x" << std::hex << int(cmd) << " addr=0x" << int(address) << std::dec
                 << " source=" << context_.source << " attempt=" << attempt << " mono_ms=" << started
                 << " tx_queue_delay_ms="
                 << (context_.queued_ms > 0 && send_start >= 0
                            ? std::max(0., send_start - context_.queued_ms)
                            : 0.)
                 << " tx_write_duration_ms=" << (written >= 0 ? written - send_start : -1)
                 << " first_byte_ms=" << (first >= 0 ? first - written : -1)
                 << " frame_completion_ms=" << (result.size && first >= 0 ? end - first : -1)
                 << " transaction_latency_ms=" << end - started << " timeout_ms=" << response_limit
                 << " response_deadline_ms=" << deadline << " rx_size=" << count
                 << " expected_size=" << (read ? (expected ? expected + 7 : -1) : (expects_ack ? 6 : 0))
                 << " last_unmatched=" << unmatched_reason << " unmatched=" << mismatches
                 << " checksum_errors=" << parser_.bad_checksum << " deadline_expired=" << timeout
                 << " retry=" << retry << " retry_exhausted=" << (!result.ok && attempt > 1 && !retry)
                 << " error_type=" << (negative ? "DEVICE_STATUS" : (reason.empty() ? "OK" : reason));
            if (hex_ || failed)
            {
                line << " tx_hex=[" << result.tx << "] rx_hex=[" << ToHex(raw, captured) << "]";
            }
            log_(warn, line.str());
        }
        if (result.ok)
        {
            return result;
        }
        result.rx = ToHex(raw, captured);
        result.error = reason;
        if (reason == "SERIAL_WRITE_FAILED")
        {
            result.error += " 发送失败 " + port_->LastError();
        }
        if (reason == "SERIAL_READ_ERROR")
        {
            result.error += " " + port_->LastError();
        }
        if (!retry)
        {
            if (attempt > 1)
            {
                result.error = "RETRY_EXHAUSTED " + result.error;
            }
            return result;
        }
    }
    return result;
}
} // namespace ds485
