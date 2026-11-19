#ifndef DS485_DRIVER_FAKE_SERVO_PORT_H
#define DS485_DRIVER_FAKE_SERVO_PORT_H

/*
 * 离线测试用：一条总线上挂一个模拟德晟舵机，实现第 1 层 Rs485Port 接口。
 *   - 解析主机发来的 F9 FF 请求，校验错误的帧不应答
 *   - 按寄存器表回复读应答 / 写短应答（含 0x59 地址列表读）
 *   - SleepMs() 推进模拟时间，舵机按设定速度转向目标角度
 *   - 可放置“物体”：闭合途中被挡住（模拟夹到物体）
 *   - 可注入故障：不应答、坏校验、分包、噪声、拒绝写入、保护位
 */

#include <cstdint>
#include <string>

#include "ds485_driver/rs485_port.h"

#define FAKE_TX_SIZE 1024

struct FakeServo
{
    uint8_t id;
    uint8_t sys_config;       // 0x11，Bit3=1 写指令有短应答
    int16_t min_limit;        // 0x19
    int16_t max_limit;        // 0x1A
    uint8_t mode;             // 0x5A
    uint8_t torque_switch;    // 0x64
    uint16_t pwm_limit;       // 0x5B
    uint16_t current_limit;   // 0x5C
    uint16_t speed_limit;     // 0x5D
    int16_t temperature;      // 0x4A
    int16_t voltage;          // 0x4B
    uint8_t fault_bits;       // 状态字节 Bit0~Bit6
    uint8_t resolved_causes;  // 测试指定已消失的原因，只有新的 0x66 才能自动清锁存
    uint8_t save_mode;
    uint8_t recovery;
    uint8_t thresholds[4][4];  // 按 0x3C~0x3F 存放原始字节
    int timed_read_size;
    uint8_t ignore_write_addr;
    uint8_t drop_write_ack_addr;
    uint8_t unreadable_addr;
    bool stale_motion_ack;
    int motion_writes;

    int32_t angle;            // 当前角度 0.1°
    int32_t target;           // 目标角度 0.1°
    uint16_t move_speed;      // °/s
    bool moving;
    bool blocked;
    int move_acc;

    bool obstacle_enabled;    // 是否有物体
    int32_t obstacle_angle;   // 物体所在角度
    int stall_fault_ms;       // 被挡住超过该时间置堵转保护位，0 不置
    int blocked_ms;

    // 故障注入
    bool no_reply;
    bool corrupt_next_reply;
    bool split_reply;
    bool add_noise;
    bool reject_switch_write;
    bool hide_speed_limit;
    bool fail_send;           // Send 返回 false（模拟透传发送失败）

    // 统计
    int frames_ok;
    int frames_bad;
    int write_count;
    int read_count;
    uint8_t last_write_addr;
};


class FakeServoPort : public Rs485Port
{
public:
    FakeServoPort();
    void Reset();

    virtual bool Send(const uint8_t *data, int size);
    virtual int Receive(uint8_t *buf, int max_size, int timeout_ms);
    virtual void Clear();
    virtual void SleepMs(int ms);
    virtual std::string Name() const { return "fake"; }
    virtual std::string LastError() const { return last_error_; }

    FakeServo servo;
    long time_ms;

private:
    void HandleFrame(const uint8_t *frame, int size);
    void ReplyRead(uint8_t addr, const uint8_t *param, int len);
    int StatusReg(uint8_t addr, uint8_t *out);
    void ReplyShort(uint8_t status);
    void PushReply(uint8_t cmd, uint8_t addr, const uint8_t *data, int len, bool is_short);
    void PushBytes(const uint8_t *data, int len);
    void Step();
    uint8_t StateByte();

    uint8_t tx_[FAKE_TX_SIZE];
    int tx_len_;
    bool split_pending_;
    std::string last_error_;
};

#endif
