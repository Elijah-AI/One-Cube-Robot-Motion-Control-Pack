#ifndef DS485_DRIVER_DS485_GRIPPER_H
#define DS485_DRIVER_DS485_GRIPPER_H

/*
 * 第 3 层：自制夹爪。把舵机角度映射成 0~255 夹爪位置，并判断到位/夹到物体。
 * 只依赖第 2 层 Ds485Servo，不依赖 SDK/ROS。
 *
 * 接口与 Jodell 夹爪一致：position 0 全开 ~ 255 全闭；torque、speed 0~255，给 0 用默认值。
 *
 * 用法：
 *   Ds485Gripper gripper(&servo);
 *   gripper.SetConfig(config);           // 张开/闭合角度等（必须按机构标定）
 *   gripper.Init();                      // 检查标定、读舵机限位和速度上限
 *   gripper.StartMove(255, 0, 0);        // 以最大扭矩、最大速度闭合
 *   while (true) {
 *       int r = gripper.Update(&status); // 每次读一次状态并判断
 *       if (r != GRIPPER_RUNNING) break;
 *   }
 */

#include <cstdint>
#include <string>

#include "ds485_driver/ds485_servo.h"

// Update 的返回值
#define GRIPPER_RUNNING  0   // 仍在运动
#define GRIPPER_ARRIVED  1   // 已到达目标位置（没有夹到物体）
#define GRIPPER_BLOCKED  2   // 未到目标就停住（判定为夹到物体，舵机保持夹持力）
#define GRIPPER_FAULT    3   // 舵机报告保护/故障
#define GRIPPER_ERROR    4   // 通信连续失败

// 夹爪配置。角度单位 0.1°，速度 °/s。
struct GripperConfig
{
    int32_t open_angle;       // position=0（全开）对应的舵机角度
    int32_t close_angle;      // position=255（全闭）对应的舵机角度；与 open_angle 相等视为未标定
    uint16_t default_pwm;     // torque=0 时使用的 PWM 上限 1~1000，默认 1000（最大扭矩）
    uint16_t max_speed;       // speed=255 对应的速度；0 = 用舵机速度上限 0x5D（最大速度）
    uint16_t default_speed;   // speed=0 时使用的速度；0 = 用最大速度
    int32_t tolerance;        // 到位容差，默认 10（1.0°）
    int32_t still_band;       // 两次采样角度变化不超过该值视为“不动”，默认 3（0.3°）
    int blocked_samples;      // 连续“不动”多少次判定为夹到物体，默认 10
    int max_read_failures;    // Update 连续读失败多少次返回 GRIPPER_ERROR，默认 3

    GripperConfig();
};


class Ds485Gripper
{
public:
    explicit Ds485Gripper(Ds485Servo *servo);

    // 修改配置后需要重新 Init
    void SetConfig(const GripperConfig &config);
    const GripperConfig &Config() const { return config_; }

    /**
     * @brief 初始化：检查标定角度、读取舵机限位/模式/速度上限，算出实际最大速度。
     *        不会让舵机运动。失败后可再次调用。
     */
    bool Init();
    bool IsReady() const { return ready_; }

    /**
     * @brief 开始运动到夹爪位置（只下发，不等待）
     * @param position  0 全开 ~ 255 全闭，中间线性插值
     * @param torque    0~255 -> PWM 上限 torque*1000/255；0 = default_pwm
     * @param speed     0~255 -> speed*最大速度/255；0 = default_speed
     * @param sent      可为空。运动指令已发出时置 true
     */
    bool StartMove(uint8_t position, uint8_t torque, uint8_t speed, bool *sent = 0);

    /**
     * @brief 读一次舵机状态并判断运动结果。StartMove 之后周期调用。
     * @param status       输出本次读到的状态，可为空
     * @param have_status  可为空。本次是否成功读到状态（偶发读失败时为 false，返回 RUNNING）
     * @return GRIPPER_RUNNING / ARRIVED / BLOCKED / FAULT / ERROR
     */
    int Update(ServoStatus *status, bool *have_status = 0);

    // 停在当前位置并锁舵（保持夹持力）
    bool Stop();

    // ================= 换算 =================
    int32_t PositionToAngle(uint8_t position) const;
    uint8_t AngleToPosition(int32_t angle) const;   // 超出范围截断到 0~255
    uint16_t TorqueToPwm(uint8_t torque) const;
    uint16_t SpeedToDps(uint8_t speed) const;       // Init 成功后有效

    // Init 算出的实际速度（°/s）
    uint16_t MaxSpeed() const { return max_speed_; }
    uint16_t DefaultSpeed() const { return default_speed_; }
    // 最近一次 StartMove 的目标角度（0.1°）
    int32_t TargetAngle() const { return target_; }

    const std::string &LastError() const { return last_error_; }
    Ds485Servo *Servo() const { return servo_; }

private:
    Ds485Servo *servo_;
    GripperConfig config_;
    bool ready_;
    uint16_t max_speed_;
    uint16_t default_speed_;

    // 到位 / 夹物判断
    int32_t target_;
    int32_t anchor_angle_;   // 判断“不动”的基准角度
    int settled_count_;      // 连续“停止且在容差内”的次数
    int still_count_;        // 连续“角度几乎不变”的次数
    int fail_count_;         // 连续读状态失败的次数

    std::string last_error_;
};

#endif
