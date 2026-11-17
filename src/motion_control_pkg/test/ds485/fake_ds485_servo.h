#ifndef __FAKE_DS485_SERVO_H
#define __FAKE_DS485_SERVO_H

/*
 * 模拟德晟舵机（第五版协议），用于离线测试 Ds485_Gripper。
 * 实现 Ds485_Transport 接口，代替机械臂末端透传：
 *   - 解析主机发来的 F9 FF 指令帧，校验错误的帧不应答
 *   - 按寄存器表回复读应答 / 写短应答
 *   - Sleep_ms() 推进模拟时间，舵机按设定速度转向目标角度
 *   - 可设置“物体”角度，闭合时在该角度被挡住（模拟夹到物体）
 *   - 可注入故障：不应答、坏校验、分包、噪声、写入拒绝、保护位
 */

#include <cstdint>

#include "Ds485_Gripper.h"

#define FAKE_TX_SIZE 1024

// 单个模拟舵机的状态
struct Fake_Servo
{
    uint8_t  id;
    uint8_t  sys_config;      // 0x11，Bit3=1 写指令有短应答
    int16_t  min_limit;       // 0x19，0.1°，0 不限
    int16_t  max_limit;       // 0x1A
    uint8_t  mode;            // 0x5A
    uint8_t  torque_switch;   // 0x64
    uint16_t pwm_limit;       // 0x5B
    uint16_t current_limit;   // 0x5C
    uint16_t speed_limit;     // 0x5D
    int16_t  temperature;     // 0x4A
    int16_t  voltage;         // 0x4B
    uint8_t  fault_bits;      // 状态字节 Bit0~Bit6

    int32_t  angle;           // 当前角度 0.1°
    int32_t  target;          // 目标角度 0.1°
    uint16_t move_speed;      // 当前运动速度 °/s
    bool     moving;          // 状态字节 Bit7
    bool     blocked;         // 被物体挡住
    int      move_acc;        // 运动步长余数

    bool     obstacle_enabled;  // 是否有物体
    int32_t  obstacle_angle;    // 物体所在角度：向目标运动途中到达此角度即被挡住
    int      stall_fault_ms;    // 被挡住超过该时间置堵转保护位，0 表示不置
    int      blocked_ms;        // 已被挡住的时间

    // ---------- 故障注入 ----------
    bool no_reply;             // 完全不应答
    bool corrupt_next_reply;   // 下一条应答前插入一帧坏校验的假应答
    bool split_reply;          // 应答分两次返回
    bool add_noise;            // 应答前插入噪声字节
    bool reject_switch_write;  // 写 0x64 时返回命令执行失败 0x40

    // ---------- 统计 ----------
    int      frames_ok;        // 收到的有效帧数
    int      frames_bad;       // 校验错误的帧数
    int      write_count;      // 写指令次数
    uint8_t  last_write_addr;  // 最后一次写的地址
    uint32_t last_channel;     // 最后一次发送使用的通道
};


class Fake_Ds485_Bus : public Ds485_Transport
{
public:
    Fake_Ds485_Bus();
    virtual ~Fake_Ds485_Bus() {}

    // 恢复某侧舵机的默认状态（角度 0，模式 1，开关 0，写应答打开）
    void Reset_Servo(int side);

    // Ds485_Transport 接口
    virtual bool Clear(int side);
    virtual bool Send(int side, uint32_t channel, const uint8_t *data, int size);
    virtual int Receive(int side, uint32_t channel, uint8_t *data, int max_size);
    virtual void Sleep_ms(int ms);

    Fake_Servo servo[DS485_SIDE_NUM];
    long time_ms;              // 模拟时间，毫秒

private:
    void Handle_Frame(int side, const uint8_t *frame, int size);
    void Reply_Read(int side, uint8_t addr);
    void Reply_Combined(int side, const uint8_t *addresses, int len);
    void Reply_Short(int side, uint8_t status);
    void Push_Reply(int side, uint8_t cmd, uint8_t addr, const uint8_t *data, int len, bool is_short);
    void Push_Bytes(int side, const uint8_t *data, int len);
    void Step(int side, int ms);
    uint8_t State_Byte(int side);

    uint8_t tx_[DS485_SIDE_NUM][FAKE_TX_SIZE];  // 待返回给主机的字节
    int tx_len_[DS485_SIDE_NUM];
    int split_left_[DS485_SIDE_NUM];            // 分包模式下第一次只返回的字节数
};

#endif
