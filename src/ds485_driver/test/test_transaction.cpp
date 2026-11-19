#include "ds485_driver/serial_transaction.h"
#include "ds485_driver/ds485_servo.h"
#include "fake_servo_port.h"
#include <algorithm>
#include <cassert>
#include <deque>
#include <functional>
#include <vector>
using Bytes = std::vector<uint8_t>;
using namespace ds485;
namespace
{
Bytes Reply(uint8_t id = 1, uint8_t cmd = 2, uint8_t addr = 0x46)
{
    Bytes b{0xf9, 0xf5, id, 7, cmd, addr, 0xff, 0xff, 0xff, 0xff, 0};
    unsigned sum = 0;
    for (size_t i = 2; i + 1 < b.size(); ++i)
    {
        sum += b[i];
    }
    b.back() = static_cast<uint8_t>(~sum);
    return b;
}
struct Wire : Rs485Port
{
    double now = 0;
    int writes = 0;
    bool fail_send = false, fail_read = false;
    std::deque<std::pair<double, Bytes>> arrivals;
    std::function<void(Wire &)> on_send;
    void Add(double delay, Bytes b)
    {
        arrivals.emplace_back(now + delay, std::move(b));
    }
    bool Send(const uint8_t *, int) override
    {
        ++writes;
        on_send(*this);
        return !fail_send;
    }
    int Receive(uint8_t *p, int, int) override
    {
        if (fail_read)
        {
            return -1;
        }
        if (arrivals.empty() || arrivals.front().first > now)
        {
            return 0;
        }
        const auto b = arrivals.front().second;
        arrivals.pop_front();
        std::copy(b.begin(), b.end(), p);
        return int(b.size());
    }
    void Clear() override
    {
        while (!arrivals.empty() && arrivals.front().first <= now)
        {
            arrivals.pop_front();
        }
    }
    void SleepMs(int ms) override
    {
        now += ms;
    }
    double NowMs() const override
    {
        return now;
    }
    std::string Name() const override
    {
        return "wire";
    }
    std::string LastError() const override
    {
        return "injected";
    }
};
} // namespace
void TestTransactions()
{
    Wire silent;
    silent.on_send = [](Wire &) {};
    SerialTransactionManager active(&silent);
    active.Timing(0, 300);
    TransactionContext cycle;
    cycle.deadline_ms = 300;
    cycle.response_limit_ms = 40;
    active.Context(cycle);
    assert(!active.Exchange(1, 2, 0x46, nullptr, 0, 4, true).ok && silent.now == 40);
    FakeServoPort device;
    Ds485Servo servo(&device, 1);
    ServoStatus snapshot{};
    servo.SetTiming(1, 10);
    assert(servo.ReadStatus(&snapshot) && snapshot.valid);
    const auto last = snapshot.sampled_ms;
    device.servo.fail_send = true;
    assert(!servo.ReadStatus(&snapshot) && !snapshot.valid && snapshot.sampled_ms == last);
    auto exercise = [](Bytes bytes, const std::string &reason)
    {
        Wire port;
        port.on_send = [&](Wire &w)
        {
            if (!bytes.empty())
            {
                w.Add(0, bytes);
            }
        };
        SerialTransactionManager t(&port);
        t.Timing(0, 12);
        auto r = t.Exchange(1, 2, 0x46, nullptr, 0, 4, true);
        assert(!r.ok && r.error == reason && port.now == 12 && port.writes == 1);
    };
    exercise({}, "RX_NO_BYTES");
    exercise({0xf9, 0xf5, 1}, "RX_PARTIAL_FRAME");
    exercise({2, 3, 4}, "RX_BAD_HEADER");
    exercise({0xf9, 0xf5, 1, 0xff}, "RX_BAD_LENGTH");
    Bytes corrupt = Reply();
    corrupt.back() ^= 1;
    exercise(corrupt, "RX_CHECKSUM_ERROR");
    exercise(Reply(2), "RX_ID_MISMATCH");
    exercise(Reply(1, 3), "RX_COMMAND_MISMATCH");
    exercise(Reply(1, 2, 0x47), "RX_ADDRESS_MISMATCH");
    Wire port;
    SerialTransactionManager t(&port);
    t.Timing(1, 12);
    t.Options(1, true);
    // 第一次读的旧包在超时后到达。第二次读前隔离，再处理噪声、错 ID 和跨块的正确帧。
    port.Add(0, Reply());
    port.on_send = [](Wire &w)
    {
        if (w.writes == 1)
        {
            w.Add(16, Reply());
            return;
        }
        auto a = Reply(2);
        auto b = Reply();
        a.insert(a.begin(), 0x24);
        a.insert(a.end(), b.begin(), b.begin() + 5);
        w.Add(1, a);
        w.Add(3, Bytes(b.begin() + 5, b.end()));
    };
    auto r = t.Exchange(1, 2, 0x46, nullptr, 0, 4, true);
    assert(r.ok && port.writes == 2 && GetI32(r.frame.data() + 6) == -1 && port.now >= 28);
    assert(t.Stats().retries == 1 && t.Stats().timeouts == 1);
    port.on_send = [](Wire &) {};
    r = t.Exchange(1, 2, 0x46, nullptr, 0, 4, true);
    assert(!r.ok && r.error == "RETRY_EXHAUSTED RX_NO_BYTES");
    int before = port.writes;
    uint8_t zero = 0;
    r = t.Exchange(1, 3, 0x64, &zero, 1, 0, false);
    assert(r.ok && r.size == 0 && port.writes == before + 1);
    r = t.Exchange(1, 3, 0x64, &zero, 1, 0, true);
    assert(!r.ok && r.sent && port.writes == before + 2);
    t.Options(0, false);
    port.on_send = [](Wire &w) { w.Add(0, {0xf9, 0xf5, 1, 2, 0, 0xfc}); };
    r = t.Exchange(1, DS485_CMD_QUERY, 0, nullptr, 0, 0, false);
    assert(r.ok && r.size == 6 && r.tx == "F9 FF 01 02 01 FB");
    port.on_send = [](Wire &) {};
    for (uint8_t cmd : {3, 4, 0x83, 0x84})
    {
        int id = cmd == 4 ? 1 : 254;
        auto sent = t.Exchange(id, cmd, 0x64, cmd == 0x84 ? nullptr : &zero, cmd == 0x84 ? 0 : 1, 0, true);
        assert(sent.ok && sent.size == 0);
    }
    r = t.Exchange(254, DS485_CMD_READ, 1, nullptr, 0, 2, true);
    assert(!r.ok && !r.sent);
    port.fail_send = true;
    r = t.Exchange(1, 3, 0x64, &zero, 1, 0, true);
    assert(!r.ok && !r.sent && r.error.find("SERIAL_WRITE_FAILED") == 0);
    port.fail_send = false;
    port.fail_read = true;
    t.Options(0, false);
    r = t.Exchange(1, 2, 0x46, nullptr, 0, 4, true);
    assert(!r.ok && r.error.find("SERIAL_READ_ERROR") == 0);
    port.fail_read = false;
    TransactionContext c;
    c.interrupted = [&] { return true; };
    t.Context(c);
    before = port.writes;
    r = t.Exchange(1, 3, 0x69, &zero, 1, 0, true);
    assert(r.error == "TX_CANCELED" && port.writes == before);
}
