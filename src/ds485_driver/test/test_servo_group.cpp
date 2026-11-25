/*
 * 离线测试：同一总线多台舵机的同步写帧格式（协议第 19~20 页）。
 */
#include "ds485_driver/ds485_servo_group.h"
#include <cassert>
#include <string>
#include <vector>

using Bytes = std::vector<uint8_t>;
using namespace ds485;

namespace
{
// 只记录发出的帧，不应答。0x83、0x04、0x84 按协议都没有应答。
struct Capture : Rs485Port
{
    double now = 0;
    bool fail = false;
    int sends = 0, fail_at = -1; // 第 fail_at 次发送（从 0 计）失败
    std::vector<Bytes> frames;
    bool Send(const uint8_t *data, int size) override
    {
        if (fail || sends++ == fail_at)
        {
            return false;
        }
        frames.emplace_back(data, data + size);
        return true;
    }
    int Receive(uint8_t *, int, int) override
    {
        return 0;
    }
    void Clear() override {}
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
        return "capture";
    }
    std::string LastError() const override
    {
        return "injected";
    }
};

// 追加校验字节：~(从 ID 起的字节和)
Bytes Seal(Bytes frame)
{
    unsigned sum = 0;
    for (size_t i = 2; i < frame.size(); ++i)
    {
        sum += frame[i];
    }
    frame.push_back(static_cast<uint8_t>(~sum));
    return frame;
}
} // namespace

void TestServoGroup()
{
    // 协议例 1：对 ID1、ID2、ID3 发送 0x64=2
    const uint8_t ids[] = {1, 2, 3};
    const uint8_t hold[] = {2, 2, 2};
    uint8_t payload[DS485_FRAME_MAX], frame[DS485_FRAME_MAX];
    int n = BuildMultiWritePayload(ids, 3, hold, 1, payload);
    assert(n == 7);
    n = BuildFrame(DS485_ID_BROADCAST, DS485_CMD_MULTI_WRITE, DS485_REG_SWITCH, payload, n, frame);
    const Bytes example{0xF9, 0xFF, 0xFE, 0x0A, 0x83, 0x64, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03, 0x02, 0x03};
    assert(Bytes(frame, frame + n) == example);
    // 协议例 3：每台 2 字节时长度为 0x0D
    const uint8_t ids3[] = {5, 7, 9};
    const uint8_t timed[] = {0x00, 0x00, 0x5A, 0x00, 0xA6, 0xFF};
    n = BuildMultiWritePayload(ids3, 3, timed, 2, payload);
    assert(n == 10);
    assert(BuildFrame(DS485_ID_BROADCAST, DS485_CMD_MULTI_WRITE, 0x65, payload, n, frame) == 17 && frame[3] == 0x0D);
    // 整帧超过 256 字节、非法 ID 均拒绝
    uint8_t many[90], wide[180] = {};
    for (int i = 0; i < 90; ++i)
    {
        many[i] = static_cast<uint8_t>(i);
    }
    assert(BuildMultiWritePayload(many, 90, wide, 2, payload) == 0);
    const uint8_t broadcast[] = {DS485_ID_BROADCAST};
    assert(BuildMultiWritePayload(broadcast, 1, hold, 1, payload) == 0);
    assert(MultiWriteAddress(DS485_REG_MOTOR_PWM) && MultiWriteAddress(DS485_REG_SWITCH));
    assert(!MultiWriteAddress(DS485_REG_MODE) && !MultiWriteAddress(DS485_REG_INTERPOLATE));

    Capture port;
    SerialTransactionManager bus(&port);
    bus.Timing(0, 20);
    Ds485Servo a(&port, 1, &bus), b(&port, 2, &bus);
    assert(&a.Transactions() == &bus && &b.Transactions() == &bus);

    // 0x83：一帧同时给两台不同方向的 PWM，不等应答
    Ds485ServoGroup group({&a, &b}, SyncWrite::MultiId);
    SyncResult r = group.WriteMotorPwm({100, -100});
    assert(r.ok && r.sent && !r.acknowledged && r.sent_ms >= 0 && port.frames.size() == 1);
    const Bytes pwm{0xF9, 0xFF, 0xFE, 0x0A, 0x83, 0x6E, 0x02, 0x01, 0x64, 0x00, 0x02, 0x9C, 0xFF, 0x02};
    assert(port.frames[0] == pwm);
    // 只写选中的舵机
    const std::vector<bool> second{false, true};
    r = group.WriteSwitch(DS485_SWITCH_BRAKE, &second);
    assert(r.ok && port.frames.size() == 2);
    assert(port.frames[1] == Seal({0xF9, 0xFF, 0xFE, 0x06, 0x83, 0x64, 0x01, 0x02, 0x01}));
    // 0x83 未开放的地址不发送
    const std::vector<std::vector<uint8_t>> motor_mode(2, std::vector<uint8_t>{2});
    r = group.Write(DS485_REG_MODE, motor_mode);
    assert(!r.ok && !r.sent && port.frames.size() == 2);
    assert(!group.WriteMotorPwm({1001, 0}).ok && port.frames.size() == 2);

    // 0x04 逐台放入缓存，0x84 广播后同时执行
    Ds485ServoGroup staged({&a, &b}, SyncWrite::Staged);
    r = staged.WriteMotorPwm({100, -100});
    assert(r.ok && r.sent && !r.acknowledged && port.frames.size() == 5);
    assert(port.frames[2] == Seal({0xF9, 0xFF, 0x01, 0x05, 0x04, 0x6E, 0x64, 0x00}));
    assert(port.frames[3] == Seal({0xF9, 0xFF, 0x02, 0x05, 0x04, 0x6E, 0x9C, 0xFF}));
    assert(port.frames[4] == Bytes({0xF9, 0xFF, 0xFE, 0x02, 0x84, 0x7B}));
    // 缓存帧发送失败时不广播执行帧
    port.fail = true;
    r = staged.WriteMotorPwm({50, 50});
    assert(!r.ok && !r.sent && port.frames.size() == 5);
    port.fail = false;
    // ID 2 的缓存帧失败：不发 0x84，并读一次 ID 1 的开关以清除它已收下的预存
    port.fail_at = port.sends + 1;
    r = staged.WriteMotorPwm({30, 30});
    assert(!r.ok && !r.sent && r.error.find("ID 2") != std::string::npos);
    assert(port.frames.size() == 7);
    assert(port.frames[5] == Seal({0xF9, 0xFF, 0x01, 0x05, 0x04, 0x6E, 0x1E, 0x00}));
    assert(port.frames[6] == Seal({0xF9, 0xFF, 0x01, 0x03, 0x02, 0x64}));
    port.fail_at = -1;

    // 各自持有事务管理器的舵机不能组成同步组
    Ds485Servo lone(&port, 3);
    Ds485ServoGroup mixed({&a, &lone}, SyncWrite::MultiId);
    assert(mixed.WriteSwitch(DS485_SWITCH_BRAKE).error == "SYNC_GROUP_INVALID");

    SyncWrite mode = SyncWrite::Sequential;
    assert(ParseSyncWrite("multi_id", &mode) && mode == SyncWrite::MultiId);
    assert(ParseSyncWrite("staged", &mode) && mode == SyncWrite::Staged);
    assert(!ParseSyncWrite("broadcast", &mode) && mode == SyncWrite::Staged);
    assert(std::string(Name(SyncWrite::Sequential)) == "sequential");
}
