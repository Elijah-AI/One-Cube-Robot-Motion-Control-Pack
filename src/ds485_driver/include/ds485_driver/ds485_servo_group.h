#ifndef DS485_DRIVER_DS485_SERVO_GROUP_H
#define DS485_DRIVER_DS485_SERVO_GROUP_H

/*
 * 第 2 层：同一条总线上多台舵机的同步写。只依赖第 1、2 层，不依赖 SDK/ROS。
 *
 * 三种方式（协议第 19~20 页）：
 *   MultiId    0x83 多 ID 写。一帧广播（ID 254）携带每台的数据，各舵机收到同一帧后同时执行，
 *              不应答。只开放 0x64 0x65 0x66 0x68 0x6E 0x6F。默认方式。
 *   Staged     逐台 0x04 同步写放入缓存，再广播 0x84 同时执行。不应答，帧数为台数 + 1。
 *   Sequential 逐台 0x03 普通写，按 0x11 Bit3 等短应答。各台生效时刻相差一笔事务。
 *
 * 用法：
 *   SerialTransactionManager bus(&port);
 *   Ds485Servo a(&port, 1, &bus), b(&port, 2, &bus);
 *   Ds485ServoGroup group({&a, &b}, SyncWrite::MultiId);
 *   auto r = group.WriteMotorPwm({100, -100});   // 一帧同时启动两台
 *
 * 无应答方式只说明报文已交给传输层，执行结果需由调用者逐台读回确认。
 */

#include <cstdint>
#include <string>
#include <vector>

#include "ds485_driver/ds485_servo.h"

namespace ds485
{
enum class SyncWrite
{
    MultiId,
    Staged,
    Sequential
};

// 配置字符串 multi_id / staged / sequential
bool ParseSyncWrite(const std::string &text, SyncWrite *mode);
const char *Name(SyncWrite mode);

struct SyncResult
{
    bool ok = false;            // 全部报文已交给传输层；顺序写还要求每台短应答成功（打开写应答时）
    bool sent = false;          // 可能已有舵机执行（生效帧已交给传输层，或顺序写已发出至少一台）
    bool acknowledged = false;  // 每台都返回了成功短应答，只有顺序写且全部打开写应答时可能为 true
    double sent_ms = -1;        // 生效帧（0x83、0x84 或最后一条 0x03）交给传输层的时刻
    std::string error;
};

class Ds485ServoGroup
{
public:
    // 所有舵机必须挂在同一条总线并共用同一个事务管理器，否则写入一律失败。
    Ds485ServoGroup(const std::vector<Ds485Servo *> &servos, SyncWrite mode);

    SyncWrite Mode() const { return mode_; }
    size_t Size() const { return servos_.size(); }

    /**
     * @brief 同一地址给每台写各自的数据
     * @param data  与构造时的舵机顺序一致，每台字节数必须相同
     * @param mask  可为空；非空时只写 mask[i] 为 true 的舵机
     * @param best_effort  只影响逐台写：false（启动用）遇首台失败即返回，由调用者整侧停止；
     *        true（停止用）每台都尝试一次，返回第一条错误。
     *        只接受 0x83 开放的运动地址，保存参数和限位等配置仍逐台写入并读回。
     *
     * 0x04 预存中途失败、或 0x84 没能发出时，已预存的舵机各读一次开关寄存器以清除缓存，
     * 防止之后的 0x84 执行残留数据。
     */
    SyncResult Write(uint8_t addr, const std::vector<std::vector<uint8_t>> &data,
                     const std::vector<bool> *mask = nullptr, bool best_effort = false);
    // 0x6E 电机定扭，每台 -1000~1000
    SyncResult WriteMotorPwm(const std::vector<int16_t> &pwm, const std::vector<bool> *mask = nullptr,
                             bool best_effort = false);
    // 0x64 扭矩开关，所有选中的舵机写同一值
    SyncResult WriteSwitch(uint8_t mode, const std::vector<bool> *mask = nullptr, bool best_effort = false);

private:
    void ClearStaged(const std::vector<size_t> &staged);

    std::vector<Ds485Servo *> servos_;
    SyncWrite mode_;
    bool shared_ = false;
};
}  // namespace ds485

#endif
