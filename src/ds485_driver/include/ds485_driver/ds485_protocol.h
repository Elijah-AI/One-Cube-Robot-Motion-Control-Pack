#ifndef DS485_DRIVER_DS485_PROTOCOL_H
#define DS485_DRIVER_DS485_PROTOCOL_H

/*
 * 第 2 层（协议部分）：德晟串口总线智能舵机 第五版协议的报文组装与解析。
 * 只处理字节，不收发、不依赖 SDK/ROS，可单独测试。
 *
 *   请求：F9 FF  ID  LEN(3+N)  CMD  ADDR  参数[N]  CHK
 *   读应答：F9 F5  ID  LEN(3+N)  CMD  ADDR  数据[N]  CHK
 *   写短应答：F9 F5  ID  0x02  STATUS  CHK
 *   CHK = ~(从 ID 到最后一个参数的字节和) & 0xFF；多字节数据小端。
 *   协议中的“扭矩”实际是驱动 PWM，0~1000 对应 0%~100%，不是 N·m。
 */

#include <cstdint>
#include <string>

// ---------------- 功能码 ----------------
#define DS485_CMD_QUERY  0x01
#define DS485_CMD_STAGE  0x04
#define DS485_CMD_MULTI_WRITE 0x83
#define DS485_CMD_APPLY  0x84
#define DS485_CMD_READ   0x02
#define DS485_CMD_WRITE  0x03

// ---------------- 寄存器地址（本工程用到的部分）----------------
#define DS485_REG_STATE         0x01  // 状态 Uint8 + 随机数 Uint8
#define DS485_REG_USER_SAVE     0x04  // 0 写用户参数后立即保存，1 仅本次上电有效
#define DS485_REG_SYS_CONFIG    0x11  // 系统配置 Uint8，Bit3=1 表示写指令有短应答
#define DS485_REG_MAX_PWM       0x13  // 已存储的最大 PWM Uint16（只读，不写）
#define DS485_REG_MIN_ANGLE     0x19  // 最小角度限位 Int16，0.1°，0 表示不限
#define DS485_REG_MAX_ANGLE     0x1A  // 最大角度限位 Int16，0.1°，0 表示不限
#define DS485_REG_RECOVERY      0x33  // 低四位：0 自动或指令解除，1 仅指令解除
#define DS485_REG_VOLTAGE_TRIP  0x3C  // 高压、低压，单位 V
#define DS485_REG_TEMP_TRIP     0x3D  // 触发温度、解除回差，单位 ℃
#define DS485_REG_CURRENT_TRIP  0x3E  // 电流 mA、持续时间（100 ms/格）
#define DS485_REG_STALL_TRIP    0x3F  // 堵转 PWM、持续时间（100 ms/格）
#define DS485_REG_ANGLE         0x46  // 实际角度 Int32，0.1°
#define DS485_REG_SPEED         0x47  // 实际速度 Int16，°/s
#define DS485_REG_CURRENT       0x48  // 电流 Int16，mA
#define DS485_REG_PWM           0x49  // 当前有符号 PWM Int16
#define DS485_REG_TEMPERATURE   0x4A  // 温度 Int16，℃
#define DS485_REG_VOLTAGE       0x4B  // 电压 Int16，mV
#define DS485_REG_ANGLE_ERROR   0x4C  // 目标角度减当前角度，Int32，0.1°
#define DS485_REG_MULTI_READ    0x59  // 地址列表读：参数为地址列表（只支持 0x46~0x4D）
#define DS485_REG_MODE          0x5A  // 控制模式 Uint8：0 自动 1 舵机 2 电机
#define DS485_REG_PWM_LIMIT     0x5B  // 运行时 PWM 上限 Uint16，0~1000（掉电恢复）
#define DS485_REG_CURRENT_LIMIT 0x5C  // 运行时电流上限 Uint16，mA（掉电恢复）
#define DS485_REG_SPEED_LIMIT   0x5D  // 运行时速度上限 Uint16，°/s（掉电恢复）
#define DS485_REG_SWITCH        0x64  // 扭矩开关 Uint8：0 释放 1 刹车 2 锁舵
#define DS485_REG_MOVE_SPEED    0x66  // 定速绝对角度：角度 Int16(0.1°) + 速度 Uint16(°/s)
#define DS485_REG_INTERPOLATE   0x69  // 主机规划的绝对角度 Int16，0.1°，20~100 Hz

#define DS485_REG_MOTOR_PWM     0x6E  // 电机模式定扭矩，Int16，-1000~1000，0 停转

// ---------------- 取值 ----------------
#define DS485_PWM_MAX         1000
#define DS485_SWITCH_RELEASE  0   // 关闭输出，自由释放
#define DS485_SWITCH_BRAKE    1   // 关闭驱动，带刹车阻尼
#define DS485_SWITCH_HOLD     2   // 打开输出，在当前位置锁舵

// 状态字节：Bit0~Bit6 为保护/故障位，Bit7 为“正在转动”
#define DS485_STATE_FAULT_MASK 0x7F
#define DS485_STATE_PROTECTION_MASK 0x1F  // 物理保护，含 Bit4 硬件故障
#define DS485_AUTO_FAULT_MASK  0x0F  // 0x33 支持电压、电流、温度、堵转四类保护
#define DS485_STATE_MOVING     0x80

// 一帧最大长度（LEN 为 1 字节，总长 = LEN + 4）
#define DS485_FRAME_MAX 256

// ID：单个舵机 0~250；254 为广播，广播帧不应答
#define DS485_ID_MAX       250
#define DS485_ID_BROADCAST 254

namespace ds485
{

// ---------------- 小端读写 ----------------
uint16_t GetU16(const uint8_t *p);
int16_t GetI16(const uint8_t *p);
int32_t GetI32(const uint8_t *p);
void PutU16(uint8_t *p, uint16_t value);
void PutI16(uint8_t *p, int16_t value);

/**
 * @brief 组装一帧请求
 * @param out 至少 DS485_FRAME_MAX 字节
 * @return 帧长度；参数过长返回 0
 */
int BuildFrame(uint8_t id, uint8_t cmd, uint8_t addr,
               const uint8_t *payload, int payload_len, uint8_t *out);

// 协议第 19 页 4.2 节：0x83 多 ID 写只开放 0x64 0x65 0x66 0x68 0x6E 0x6F
bool MultiWriteAddress(uint8_t addr);

/**
 * @brief 组装 0x83 多 ID 写的参数序列：[每台数据字节数][ID1][数据1][ID2][数据2]...
 *        整帧以广播 ID 254 发出，各舵机收到同一帧后同时执行，均不应答。
 * @param ids    目标 ID，0~250
 * @param data   按 ids 顺序连续存放，每台 width 字节（小端）
 * @param out    至少 DS485_FRAME_MAX 字节
 * @return 参数序列长度；参数非法或整帧超过 256 字节返回 0
 */
int BuildMultiWritePayload(const uint8_t *ids, int count, const uint8_t *data, int width, uint8_t *out);

// 把状态字节中的故障位写成中文，例如 "过流保护、堵转/过载保护"；无故障返回 "无故障"
std::string FaultText(uint8_t state);

// 字节转十六进制字符串，例如 "F9 FF 01 04 02 46 B3"，调试日志用
std::string ToHex(const uint8_t *data, int size);


/*
 * 应答帧解析器：处理分包、帧头前的噪声、坏校验。
 * 用法：每收到一批字节 Push 一次，然后循环 Pop 直到返回 0。
 */
class FrameParser
{
public:
    FrameParser();

    void Reset();
    // 追加收到的字节。缓存满时丢弃旧数据。
    void Push(const uint8_t *data, int size);
    /**
     * @brief 取出一帧校验正确的应答（F9 F5 开头）
     * @param frame 至少 DS485_FRAME_MAX 字节
     * @return 帧长度；没有完整帧返回 0
     */
    int Pop(uint8_t *frame);

    // 因坏校验/长度非法被丢弃的字节数（调试用）
    long DroppedBytes() const { return dropped_; }
    int Pending() const { return len_; }
    unsigned bad_header = 0, bad_length = 0, bad_checksum = 0;

private:
    void Drop(int count);

    uint8_t buf_[DS485_FRAME_MAX * 2];
    int len_;
    long dropped_;
};

}  // namespace ds485

#endif
