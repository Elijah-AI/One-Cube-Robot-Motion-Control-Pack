/*
 * 第 3 层：自制夹爪。
 * 到位/夹物判断与 motion_control_pkg/Ds485_Gripper.cpp（已在机器人上测试）保持一致。
 */
#include "ds485_driver/ds485_gripper.h"

#include <stdio.h>

static int32_t Abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

GripperConfig::GripperConfig()
{
    open_angle = 0;
    close_angle = 0;
    default_pwm = DS485_PWM_MAX;
    max_speed = 0;
    default_speed = 0;
    tolerance = 10;
    still_band = 3;
    blocked_samples = 10;
    max_read_failures = 3;
}


Ds485Gripper::Ds485Gripper(Ds485Servo *servo)
{
    servo_ = servo;
    ready_ = false;
    max_speed_ = 0;
    default_speed_ = 0;
    target_ = 0;
    anchor_angle_ = 0;
    settled_count_ = 0;
    still_count_ = 0;
    fail_count_ = 0;
}

void Ds485Gripper::SetConfig(const GripperConfig &config)
{
    config_ = config;
    ready_ = false;
}


// ======================= 初始化 =======================

bool Ds485Gripper::Init()
{
    ready_ = false;
    char text[200];

    if (config_.open_angle == config_.close_angle)
    {
        last_error_ = "张开/闭合角度未标定（open == close），请先在配置文件中填写 open_deg / close_deg";
        return false;
    }
    if (config_.open_angle < -32768 || config_.open_angle > 32767 ||
        config_.close_angle < -32768 || config_.close_angle > 32767)
    {
        last_error_ = "张开/闭合角度超出协议范围 -3276.8°~3276.7°";
        return false;
    }
    if (config_.default_pwm < 1 || config_.default_pwm > DS485_PWM_MAX)
    {
        last_error_ = "default_pwm 范围为 1~1000";
        return false;
    }

    // 1. 读舵机限位、模式、速度上限
    if (!servo_->Probe())
    {
        last_error_ = servo_->LastError();
        return false;
    }
    if (servo_->Mode() != 0 && servo_->Mode() != 1)
    {
        snprintf(text, sizeof(text), "舵机控制模式为 %d（电机模式），请先人工切回舵机模式", servo_->Mode());
        last_error_ = text;
        return false;
    }

    // 2. 标定角度必须在舵机限位内（限位为 0 表示该侧不限）
    int16_t min_limit = servo_->MinAngle();
    int16_t max_limit = servo_->MaxAngle();
    int32_t angles[2] = {config_.open_angle, config_.close_angle};
    for (int i = 0; i < 2; i++)
    {
        if ((min_limit != 0 && angles[i] < min_limit) || (max_limit != 0 && angles[i] > max_limit))
        {
            snprintf(text, sizeof(text), "标定角度 %.1f° 超出舵机限位 %.1f°~%.1f°",
                     angles[i] / 10.0, min_limit / 10.0, max_limit / 10.0);
            last_error_ = text;
            return false;
        }
    }

    // 3. 实际最大速度 = min(配置 max_speed, 舵机速度上限)；配置为 0 时直接用舵机上限
    uint16_t limit = servo_->SpeedLimit();
    uint16_t max_speed = config_.max_speed;
    if (max_speed == 0 || (limit > 0 && max_speed > limit))
    {
        max_speed = limit;
    }
    if (max_speed == 0)
    {
        last_error_ = "读不到舵机速度上限 0x5D 且未配置 max_speed，或速度上限为 0，无法运动";
        return false;
    }
    uint16_t default_speed = config_.default_speed;
    if (default_speed == 0 || default_speed > max_speed)
    {
        default_speed = max_speed;
    }
    max_speed_ = max_speed;
    default_speed_ = default_speed;

    ready_ = true;
    last_error_.clear();
    return true;
}


// ======================= 换算 =======================

int32_t Ds485Gripper::PositionToAngle(uint8_t position) const
{
    int64_t diff = (int64_t)config_.close_angle - (int64_t)config_.open_angle;
    return (int32_t)(config_.open_angle + (diff * position) / 255);
}

uint8_t Ds485Gripper::AngleToPosition(int32_t angle) const
{
    int64_t den = (int64_t)config_.close_angle - (int64_t)config_.open_angle;
    if (den == 0)
    {
        return 0;
    }
    int64_t num = ((int64_t)angle - (int64_t)config_.open_angle) * 255;
    if (den < 0)
    {
        num = -num;
        den = -den;
    }
    // 四舍五入后截断到 0~255
    int64_t pos = num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
    if (pos < 0)
    {
        pos = 0;
    }
    if (pos > 255)
    {
        pos = 255;
    }
    return (uint8_t)pos;
}

uint16_t Ds485Gripper::TorqueToPwm(uint8_t torque) const
{
    if (torque == 0)
    {
        return config_.default_pwm;
    }
    return (uint16_t)((torque * 1000 + 127) / 255);
}

uint16_t Ds485Gripper::SpeedToDps(uint8_t speed) const
{
    if (speed == 0)
    {
        return default_speed_;
    }
    uint16_t dps = (uint16_t)((speed * (uint32_t)max_speed_ + 127) / 255);
    return dps < 1 ? 1 : dps;
}


// ======================= 运动 =======================

bool Ds485Gripper::StartMove(uint8_t position, uint8_t torque, uint8_t speed, bool *sent)
{
    if (sent != 0)
    {
        *sent = false;
    }
    if (!ready_)
    {
        last_error_ = "夹爪未初始化或初始化失败：" + last_error_;
        return false;
    }

    // 记录运动前的位置。MoveAngle 确认保护策略后，依次设置本次 PWM、锁舵、下发新目标。
    ServoStatus status;
    if (!servo_->ReadStatus(&status))
    {
        last_error_ = servo_->LastError();
        return false;
    }

    int32_t target = PositionToAngle(position);
    uint16_t speed_dps = SpeedToDps(speed);
    uint16_t pwm = TorqueToPwm(torque);
    if (!servo_->MoveAngle(target, speed_dps, sent, pwm))
    {
        last_error_ = servo_->LastError();
        return false;
    }

    target_ = target;
    anchor_angle_ = status.angle;
    settled_count_ = 0;
    still_count_ = 0;
    fail_count_ = 0;
    last_error_.clear();
    return true;
}

int Ds485Gripper::Update(ServoStatus *status_out, bool *have_status)
{
    if (have_status != 0)
    {
        *have_status = false;
    }
    ServoStatus status;
    if (!servo_->ReadStatus(&status))
    {
        fail_count_++;
        last_error_ = servo_->LastError();
        // 偶尔一次读失败不算结束，连续失败才返回错误
        return fail_count_ >= config_.max_read_failures ? GRIPPER_ERROR : GRIPPER_RUNNING;
    }
    fail_count_ = 0;
    last_error_.clear();
    if (status_out != 0)
    {
        *status_out = status;
    }
    if (have_status != 0)
    {
        *have_status = true;
    }

    // 1. 任一保护位 -> 故障
    if (status.fault)
    {
        char text[60];
        snprintf(text, sizeof(text), "运动中舵机状态异常 0x%02X（", status.state);
        last_error_ = text + ds485::FaultText(status.state) + "）";
        return GRIPPER_FAULT;
    }

    int32_t error = Abs32(status.angle - target_);

    // 2. 到位：转动位清零且在容差内，连续两次确认
    if (!status.moving && error <= config_.tolerance)
    {
        settled_count_++;
    }
    else
    {
        settled_count_ = 0;
    }
    if (settled_count_ >= 2)
    {
        return GRIPPER_ARRIVED;
    }

    // 3. 角度连续多次几乎不变：在容差内算到位，否则判定夹到物体。
    //    与“基准角度”比较而不是与上一次比较，慢速运动时位移会累积超出 still_band，不会误判。
    if (Abs32(status.angle - anchor_angle_) <= config_.still_band)
    {
        still_count_++;
    }
    else
    {
        anchor_angle_ = status.angle;   // 还在动，重新取基准
        still_count_ = 0;
    }
    if (still_count_ >= config_.blocked_samples)
    {
        return error <= config_.tolerance ? GRIPPER_ARRIVED : GRIPPER_BLOCKED;
    }
    return GRIPPER_RUNNING;
}

bool Ds485Gripper::Stop()
{
    if (!servo_->SetSwitch(DS485_SWITCH_HOLD))
    {
        last_error_ = servo_->LastError();
        return false;
    }
    return true;
}
