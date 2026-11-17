#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "Ds485_Gripper.h"

// 固定协议样本独立于模拟舵机，防止收发两端同时写错布局却仍通过测试。
class RecordedBus : public Ds485_Transport
{
public:
    enum class ReplyMode { normal, fragmented, bad_checksum, short_data, wrong_id };
    ReplyMode mode = ReplyMode::normal;
    int reads = 0;
    int writes = 0;
    std::vector<uint8_t> pending;

    bool Clear(int) override
    {
        pending.clear();
        return true;
    }

    bool Send(int side, uint32_t channel, const uint8_t *data, int size) override
    {
        assert(side == DS485_SIDE_LEFT);
        assert(channel == 2);
        if (data[4] != 2)
        {
            ++writes;
            return false;
        }
        ++reads;
        const uint8_t state_request[] = {0xf9, 0xff, 1, 3, 2, 1, 0xf8};
        const uint8_t combined_request[] = {
            0xf9, 0xff, 1, 7, 2, 0x59, 0x46, 0x47, 0x48, 0x49, 0x7e
        };
        if (data[5] == 1)
        {
            assert(size == sizeof(state_request));
            assert(std::memcmp(data, state_request, size) == 0);
            pending = {0xf9, 0xf5, 1, 5, 2, 1, 0, 0, 0xf6};
            return true;
        }
        assert(size == sizeof(combined_request));
        assert(std::memcmp(data, combined_request, size) == 0);
        // angle=-27、speed=-5、current=-123、PWM=-7；小端有符号字段。
        pending = {0xf9, 0xf5, 1, 0x0d, 2, 0x59,
                   0xe5, 0xff, 0xff, 0xff, 0xfb, 0xff,
                   0x85, 0xff, 0xf9, 0xff, 0x3e};
        if (mode == ReplyMode::bad_checksum)
        {
            pending.back() ^= 1;
        }
        else if (mode == ReplyMode::short_data || mode == ReplyMode::wrong_id)
        {
            if (mode == ReplyMode::short_data)
            {
                pending.erase(pending.end() - 2);
                --pending[3];
            }
            else
            {
                pending[2] = 2;
            }
            unsigned sum = 0;
            for (std::size_t i = 2; i + 1 < pending.size(); ++i)
            {
                sum += pending[i];
            }
            pending.back() = static_cast<uint8_t>(~sum);
        }
        return true;
    }

    int Receive(int, uint32_t, uint8_t *data, int max_size) override
    {
        int count = std::min<int>(pending.size(), max_size);
        if (mode == ReplyMode::fragmented)
        {
            count = std::min(count, 3);
        }
        std::copy_n(pending.begin(), count, data);
        pending.erase(pending.begin(), pending.begin() + count);
        return count;
    }

    void Sleep_ms(int) override {}
};

int main()
{
    const RecordedBus::ReplyMode cases[] = {
        RecordedBus::ReplyMode::normal, RecordedBus::ReplyMode::fragmented,
        RecordedBus::ReplyMode::bad_checksum, RecordedBus::ReplyMode::short_data,
        RecordedBus::ReplyMode::wrong_id
    };
    for (const auto mode : cases)
    {
        RecordedBus bus;
        bus.mode = mode;
        Ds485_Gripper gripper(&bus);
        gripper.Ds485_Config(DS485_SIDE_LEFT, 2, 1);
        gripper.Ds485_Set_Bus_Timing(DS485_SIDE_LEFT, 0, 30);
        Ds485_status state{};
        const bool ok = gripper.Ds485_Status(DS485_SIDE_LEFT, &state);
        const bool valid = mode == RecordedBus::ReplyMode::normal ||
            mode == RecordedBus::ReplyMode::fragmented;
        assert(ok == valid);
        assert(bus.reads == 2);
        assert(bus.writes == 0);
        if (valid)
        {
            assert(state.angle == -27);
            assert(state.speed == -5);
            assert(state.current == -123);
            assert(state.pwm == -7);
        }
    }
    std::puts("DS485 combined read: 5 scenarios passed");
}
