/*
 * 离线测试：第 2 层（协议 + 舵机）和第 3 层（夹爪），用模拟舵机代替真实总线。
 * 不需要 ROS、机械臂、舵机。
 *   colcon test --packages-select ds485_driver   或   sh test/build.sh
 */
#include <stdio.h>
#include <string.h>

#include "ds485_driver/ds485_gripper.h"
#include "ds485_driver/ds485_protocol.h"
#include "ds485_driver/ds485_servo.h"
#include "fake_servo_port.h"

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) {                                                       \
            g_failed++;                                                      \
            printf("  [失败] %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
        }                                                                    \
    } while (0)

// 标定好的夹爪配置：张开 -2.8°，闭合 27.2°（与左夹爪实测一致）
static GripperConfig Left_Config()
{
    GripperConfig c;
    c.open_angle = -28;
    c.close_angle = 272;
    return c;
}

// 循环 Update 直到结束，每次间隔 20ms（模拟时间）。返回 GRIPPER_xxx，超过 max_ms 返回 -1
static int Run_Until_Done(Ds485Gripper &gripper, FakeServoPort &port, ServoStatus *last, int max_ms)
{
    for (int t = 0; t < max_ms; t += 20)
    {
        int code = gripper.Update(last);
        if (code != GRIPPER_RUNNING)
        {
            return code;
        }
        port.SleepMs(20);
    }
    return -1;
}


// ======================= 协议 =======================

static void Test_Build_Frame()
{
    printf("组帧\n");
    uint8_t frame[DS485_FRAME_MAX];
    int n = ds485::BuildFrame(1, DS485_CMD_READ, DS485_REG_ANGLE, 0, 0, frame);
    // 01+03+02+46 = 0x4C，取反 = 0xB3
    const uint8_t expect[7] = {0xF9, 0xFF, 0x01, 0x03, 0x02, 0x46, 0xB3};
    CHECK(n == 7);
    CHECK(memcmp(frame, expect, 7) == 0);
    CHECK(ds485::ToHex(frame, 3) == "F9 FF 01");

    uint8_t payload[2] = {0xE8, 0x03};   // 1000
    n = ds485::BuildFrame(1, DS485_CMD_WRITE, DS485_REG_PWM_LIMIT, payload, 2, frame);
    CHECK(n == 9);
    CHECK(frame[3] == 5);
    uint32_t sum = 0;
    for (int i = 2; i < n; i++) sum += frame[i];
    CHECK((sum & 0xFF) == 0xFF);
    CHECK(ds485::BuildFrame(1, 3, 0, payload, 250, frame) == 0);   // 参数过长
}

static void Test_Parser()
{
    printf("应答解析（噪声/分包/坏校验）\n");
    ds485::FrameParser parser;
    uint8_t frame[DS485_FRAME_MAX];
    // 正确的短应答 F9 F5 01 02 00 CHK：01+02+00=3，取反 0xFC
    const uint8_t good[6] = {0xF9, 0xF5, 0x01, 0x02, 0x00, 0xFC};
    const uint8_t noise[4] = {0x55, 0xF9, 0x00, 0x12};
    const uint8_t bad[6] = {0xF9, 0xF5, 0x01, 0x02, 0x00, 0xFD};

    parser.Push(noise, 4);
    parser.Push(bad, 6);
    parser.Push(good, 3);            // 前半段
    CHECK(parser.Pop(frame) == 0);   // 还没收全
    parser.Push(good + 3, 3);        // 后半段
    CHECK(parser.Pop(frame) == 6);
    CHECK(memcmp(frame, good, 6) == 0);
    CHECK(parser.Pop(frame) == 0);
    CHECK(parser.DroppedBytes() > 0);

    CHECK(ds485::FaultText(0x00) == "无故障");
    CHECK(ds485::FaultText(0x0A) == "过流保护、堵转/过载保护");
    CHECK(ds485::FaultText(0x80) == "无故障");   // Bit7 只是“正在转动”
}


// ======================= 舵机 =======================

static void Test_Servo_Probe_Status()
{
    printf("舵机探测与状态读取\n");
    FakeServoPort port;
    port.servo.angle = 123;
    Ds485Servo servo(&port, 1);
    CHECK(servo.Probe());
    CHECK(servo.MinAngle() == -1800 && servo.MaxAngle() == 1800);
    CHECK(servo.SpeedLimit() == 360);
    CHECK(servo.Mode() == 1);

    ServoStatus st;
    CHECK(servo.ReadStatus(&st));
    CHECK(st.angle == 123);
    CHECK(!st.moving && !st.fault);
    CHECK(servo.LastRx().substr(0, 5) == "F9 F5");

    ServoInfo info;
    CHECK(servo.ReadInfo(&info));
    CHECK(info.write_ack);
    CHECK(info.pwm_limit == 1000 && info.speed_limit == 360 && info.voltage == 23941);

    // 应答被噪声、坏校验、分包干扰时仍能读出
    port.servo.add_noise = true;
    port.servo.corrupt_next_reply = true;
    port.servo.split_reply = true;
    CHECK(servo.ReadStatus(&st));
    CHECK(st.angle == 123);
}

static void Test_Servo_Move()
{
    printf("舵机定速运动\n");
    FakeServoPort port;
    Ds485Servo servo(&port, 1);
    bool sent = false;
    CHECK(servo.MoveAngle(300, 100, &sent));      // 30.0°，100°/s
    CHECK(sent);
    CHECK(port.servo.torque_switch == DS485_SWITCH_HOLD);   // 自动打开锁舵
    port.SleepMs(400);
    CHECK(port.servo.angle == 300);

    // 拒绝：超限、超速、速度 0、保护位、电机模式
    CHECK(!servo.MoveAngle(1900, 100));
    CHECK(!servo.MoveAngle(0, 400));
    CHECK(!servo.MoveAngle(0, 0));
    port.servo.fault_bits = 0x04;
    CHECK(!servo.MoveAngle(0, 100, &sent));
    CHECK(!sent);
    CHECK(servo.LastError().find("过温保护") != std::string::npos);
    port.servo.fault_bits = 0;
    port.servo.mode = 2;
    CHECK(servo.Probe());
    CHECK(!servo.MoveAngle(0, 100));
}

static void Test_Servo_Write()
{
    printf("舵机写入（短应答/无应答/拒绝/超时）\n");
    FakeServoPort port;
    Ds485Servo servo(&port, 1);
    CHECK(servo.SetPwmLimit(500));
    CHECK(port.servo.pwm_limit == 500);
    CHECK(!servo.SetPwmLimit(1001));

    // 写应答关闭：只发送，靠读回确认
    port.servo.sys_config = 0;
    CHECK(servo.Probe());
    CHECK(servo.SetPwmLimit(700));
    CHECK(port.servo.pwm_limit == 700);

    // 舵机拒绝写入
    port.servo.sys_config = 0x08;
    CHECK(servo.Probe());
    port.servo.reject_switch_write = true;
    CHECK(!servo.SetSwitch(DS485_SWITCH_BRAKE));
    CHECK(servo.LastError().find("被拒绝") != std::string::npos);
    port.servo.reject_switch_write = false;

    // 修改速度上限后 Probe 缓存跟着更新
    CHECK(servo.SetSpeedLimit(200));
    CHECK(servo.SpeedLimit() == 200);

    // 不应答 -> 超时，且不重发
    port.servo.no_reply = true;
    int writes = port.servo.frames_ok;
    CHECK(!servo.SetSwitch(DS485_SWITCH_HOLD));
    CHECK(port.servo.frames_ok == writes + 1);
    CHECK(servo.LastError().find("结果未知") != std::string::npos);
    port.servo.no_reply = false;

    // 发送失败
    port.servo.fail_send = true;
    CHECK(!servo.SetSwitch(DS485_SWITCH_HOLD));
    CHECK(servo.LastError().find("发送失败") != std::string::npos);
    port.servo.fail_send = false;

    // 原始寄存器读：0x59 地址列表读
    uint8_t data[DS485_FRAME_MAX];
    int got = 0;
    const uint8_t list[2] = {DS485_REG_ANGLE, DS485_REG_VOLTAGE};
    CHECK(servo.ReadReg(DS485_REG_MULTI_READ, data, 0, list, 2, &got));
    CHECK(got == 6);
    CHECK(ds485::GetI16(data + 4) == 23941);
    CHECK(!servo.ReadReg(0x7F, data, 0));      // 未开放的地址
}


// ======================= 夹爪 =======================

static void Test_Protection_Config()
{
    printf("保护阈值、保留位、存储和三/四字节固件\n");
    FakeServoPort port;
    Ds485Servo servo(&port, 1);
    ServoProtectionConfig config;
    config.voltage_high_v = 30;
    config.voltage_low_v = 5;
    config.temperature_trip_c = 60;
    config.temperature_hysteresis_c = 5;
    config.current_ma = 6000;
    config.current_detect_ms = 200;
    config.stall_pwm = 600;
    config.stall_detect_ms = 3000;
    port.servo.recovery = 0xA0;
    CHECK(servo.SetProtectionConfig(config));
    CHECK(servo.EnsureProtection());
    CHECK(port.servo.write_count == 0);            // 左手已存值一致，初始化零写入
    int reads = port.servo.read_count;
    CHECK(servo.EnsureProtection());
    CHECK(port.servo.read_count == reads);

    port.servo.recovery = 0xAF;
    config.voltage_low_v = -1;                    // 单独配置高压时保留原低压
    config.voltage_high_v = 29;
    config.temperature_hysteresis_c = 6;
    config.current_ma = 4321;
    config.current_detect_ms = 1200;
    config.stall_pwm = 700;
    config.stall_detect_ms = 2000;
    CHECK(servo.SetProtectionConfig(config));
    CHECK(servo.EnsureProtection());
    CHECK(port.servo.write_count == 5 && port.servo.motion_writes == 0);
    CHECK(port.servo.recovery == 0xA0 && port.servo.thresholds[0][1] == 5);
    CHECK(port.servo.thresholds[0][0] == 29 && port.servo.thresholds[1][1] == 6);
    CHECK(ds485::GetU16(port.servo.thresholds[2]) == 4321 && port.servo.thresholds[2][2] == 12);
    CHECK(ds485::GetU16(port.servo.thresholds[3]) == 700 && port.servo.thresholds[3][2] == 20);

    config.voltage_high_v = 28;
    config.stall_detect_ms = 25600;
    CHECK(servo.SetProtectionConfig(config));
    CHECK(!servo.EnsureProtection());
    CHECK(port.servo.write_count == 5 && port.servo.thresholds[0][0] == 29);
    port.servo.timed_read_size = 4;
    config.current_detect_ms = 30000;
    CHECK(servo.SetProtectionConfig(config));
    CHECK(servo.EnsureProtection());
    CHECK(ds485::GetU16(port.servo.thresholds[2] + 2) == 300);
    CHECK(ds485::GetU16(port.servo.thresholds[3] + 2) == 256);

    config.current_detect_ms = 125;
    CHECK(!servo.SetProtectionConfig(config));
    config.current_detect_ms = 30000;
    config.stall_pwm = 65536;
    CHECK(!servo.SetProtectionConfig(config));

    config = ServoProtectionConfig();
    config.auto_recover = false;
    port.servo.save_mode = 1;
    port.servo.drop_write_ack_addr = DS485_REG_RECOVERY;
    int writes = port.servo.write_count;
    CHECK(servo.SetProtectionConfig(config));
    CHECK(servo.EnsureProtection());              // ACK 丢失，读回证明已写入
    CHECK(port.servo.recovery == 0xAF && port.servo.write_count == writes + 1);
    CHECK(servo.EnsureProtection());
    CHECK(port.servo.write_count == writes + 1);  // 未开保存时也只写差异
    port.servo.recovery = 0xA0;                   // 模拟舵机单独重启，节点未看到掉线
    CHECK(servo.EnsureProtection());
    CHECK(port.servo.recovery == 0xAF && port.servo.write_count == writes + 2);
    port.servo.drop_write_ack_addr = 0;
    port.servo.ignore_write_addr = DS485_REG_RECOVERY;
    config.auto_recover = true;
    CHECK(servo.SetProtectionConfig(config));
    bool sent = true;
    CHECK(!servo.MoveAngle(50, 30, &sent));
    CHECK(!sent && port.servo.motion_writes == 0);
    CHECK(servo.LastError().find("读回不一致") != std::string::npos);

    port.servo.ignore_write_addr = 0;
    port.servo.save_mode = 0;
    CHECK(servo.EnsureProtection());
    port.servo.no_reply = true;
    uint8_t state[DS485_FRAME_MAX];
    CHECK(!servo.ReadReg(DS485_REG_STATE, state, 2));
    port.servo.no_reply = false;
    port.servo.recovery = 0xAF;
    CHECK(servo.EnsureProtection());              // 断线后旧缓存失效
    CHECK(port.servo.recovery == 0xA0);
    port.servo.timed_read_size = 2;
    config.current_ma = 4200;
    config.voltage_high_v = 26;
    CHECK(servo.SetProtectionConfig(config));
    writes = port.servo.write_count;
    CHECK(!servo.EnsureProtection());
    CHECK(port.servo.write_count == writes);      // 任一布局未知，不先写其他阈值
}

static void Test_Protection_Recovery()
{
    printf("故障采样、命令触发自动恢复和运动确认\n");
    FakeServoPort port;
    Ds485Servo servo(&port, 1);
    Ds485Gripper gripper(&servo);
    gripper.SetConfig(Left_Config());
    ServoProtectionConfig config;
    CHECK(servo.SetProtectionConfig(config));
    CHECK(gripper.Init());
    int events = 0;
    std::string log;
    servo.SetFaultLogger([&](const std::string &text) { ++events; log = text; });
    port.servo.fault_bits = 0x04;
    port.servo.temperature = 70;
    uint8_t state[DS485_FRAME_MAX];
    CHECK(servo.ReadReg(DS485_REG_STATE, state, 2));
    CHECK(events == 1 && log.find("过温保护") != std::string::npos);
    CHECK(log.find("temperature_C=70 voltage_mV=23941 current_mA=50 pwm=0") != std::string::npos);
    CHECK(servo.LastTx().find("02 01") != std::string::npos && servo.LastRx().find("02 01 04") != std::string::npos);
    CHECK(servo.ReadReg(DS485_REG_STATE, state, 2));
    CHECK(events == 1 && port.servo.write_count == 0);

    bool sent = false;
    CHECK(!gripper.StartMove(255, 128, 64, &sent)); // 故障原因仍在，固件不接受运动
    CHECK(sent && !port.servo.moving && port.servo.motion_writes == 1);
    CHECK(events == 1);
    port.servo.resolved_causes = 0x04;
    port.servo.temperature = 40;
    CHECK(servo.ReadReg(DS485_REG_STATE, state, 2));
    CHECK(port.servo.fault_bits == 0x04 && port.servo.motion_writes == 1); // 空闲轮询不清故障
    port.servo.stale_motion_ack = true;
    CHECK(gripper.StartMove(255, 128, 64, &sent));
    CHECK(sent && port.servo.fault_bits == 0 && port.servo.motion_writes == 2);
    ServoStatus status;
    CHECK(Run_Until_Done(gripper, port, &status, 3000) == GRIPPER_ARRIVED);

    port.servo.fault_bits = 0x18;                // 混合硬件故障禁止下发
    port.servo.unreadable_addr = DS485_REG_VOLTAGE;
    CHECK(!servo.MoveAngle(0, 100, &sent));
    CHECK(!sent && port.servo.motion_writes == 2 && events == 2);
    CHECK(log.find("voltage_mV=不可读") != std::string::npos && log.find("current_mA=50") != std::string::npos);
    CHECK(servo.LastError().find("硬件") != std::string::npos);

    config.auto_recover = false;
    CHECK(servo.SetProtectionConfig(config));
    port.servo.fault_bits = 0x04;
    CHECK(!servo.MoveAngle(0, 100, &sent));
    CHECK(!sent && (port.servo.recovery & 15) == 15 && port.servo.motion_writes == 2);

    config.auto_recover = true;
    port.servo.sys_config = 0;                   // 无写 ACK，0x01 加位置/偏差确认结果
    CHECK(servo.Probe());
    CHECK(servo.SetProtectionConfig(config));
    CHECK(servo.MoveAngle(0, 100, &sent));
    CHECK(sent && port.servo.motion_writes == 3 && port.servo.fault_bits == 0);
    port.servo.fault_bits = 0x02;
    CHECK(!servo.MoveAngle(100, 100, &sent));    // 无 ACK 且故障持续时不会误报已解除
    CHECK(sent && port.servo.motion_writes == 4 && port.servo.fault_bits == 2);
    port.servo.sys_config = 0x08;
    port.servo.resolved_causes = 0x02;
    port.servo.drop_write_ack_addr = DS485_REG_MOVE_SPEED;
    CHECK(servo.Probe());
    CHECK(!servo.MoveAngle(100, 100, &sent));
    CHECK(sent && port.servo.motion_writes == 5 && servo.LastError().find("结果未知") != std::string::npos);
}

static void Test_Gripper_Init()
{
    printf("夹爪初始化\n");
    FakeServoPort port;
    Ds485Servo servo(&port, 1);
    Ds485Gripper gripper(&servo);

    CHECK(!gripper.Init());                        // 未标定
    CHECK(gripper.LastError().find("未标定") != std::string::npos);
    CHECK(!gripper.StartMove(255, 0, 0));

    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());
    CHECK(gripper.MaxSpeed() == 360);              // 0 = 舵机速度上限
    CHECK(gripper.DefaultSpeed() == 360);

    GripperConfig c = Left_Config();
    c.max_speed = 100;
    c.default_speed = 50;
    gripper.SetConfig(c);
    CHECK(gripper.Init());
    CHECK(gripper.MaxSpeed() == 100 && gripper.DefaultSpeed() == 50);

    // 舵机读不到速度上限：max_speed=0 时失败，配置了 max_speed 时可用
    port.servo.hide_speed_limit = true;
    gripper.SetConfig(Left_Config());
    CHECK(!gripper.Init());
    c = Left_Config();
    c.max_speed = 120;
    gripper.SetConfig(c);
    CHECK(gripper.Init());
    CHECK(gripper.MaxSpeed() == 120);
    port.servo.hide_speed_limit = false;

    // 标定角度超出舵机限位
    port.servo.max_limit = 200;
    gripper.SetConfig(Left_Config());
    CHECK(!gripper.Init());
}

static void Test_Gripper_Mapping()
{
    printf("夹爪换算\n");
    FakeServoPort port;
    Ds485Servo servo(&port, 1);
    Ds485Gripper gripper(&servo);
    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());

    CHECK(gripper.PositionToAngle(0) == -28);
    CHECK(gripper.PositionToAngle(255) == 272);
    CHECK(gripper.AngleToPosition(-28) == 0);
    CHECK(gripper.AngleToPosition(272) == 255);
    CHECK(gripper.AngleToPosition(1000) == 255);   // 截断
    CHECK(gripper.AngleToPosition(-500) == 0);
    CHECK(gripper.TorqueToPwm(0) == 1000);         // 默认最大扭矩
    CHECK(gripper.TorqueToPwm(255) == 1000);
    CHECK(gripper.TorqueToPwm(128) == 502);
    CHECK(gripper.SpeedToDps(0) == 360);           // 默认最大速度
    CHECK(gripper.SpeedToDps(255) == 360);
    CHECK(gripper.SpeedToDps(128) == 181);
    CHECK(gripper.SpeedToDps(1) == 1);

    // 张开角度大于闭合角度（反向安装）
    GripperConfig c;
    c.open_angle = 500;
    c.close_angle = 100;
    gripper.SetConfig(c);
    CHECK(gripper.AngleToPosition(500) == 0);
    CHECK(gripper.AngleToPosition(100) == 255);
    CHECK(gripper.PositionToAngle(255) == 100);
}

static void Test_Gripper_Close_Open()
{
    printf("夹爪闭合/张开到位\n");
    FakeServoPort port;
    port.servo.angle = -28;
    Ds485Servo servo(&port, 1);
    Ds485Gripper gripper(&servo);
    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());

    port.servo.pwm_limit = 50;                     // 实机原来 50 推不动
    CHECK(gripper.StartMove(255, 0, 0));
    CHECK(port.servo.pwm_limit == 1000);           // torque=0 写入最大扭矩
    ServoStatus st;
    CHECK(Run_Until_Done(gripper, port, &st, 3000) == GRIPPER_ARRIVED);
    CHECK(port.servo.angle == 272);
    CHECK(gripper.AngleToPosition(st.angle) == 255);

    CHECK(gripper.StartMove(0, 128, 64));
    CHECK(port.servo.pwm_limit == 502);
    CHECK(Run_Until_Done(gripper, port, &st, 5000) == GRIPPER_ARRIVED);
    CHECK(port.servo.angle == -28);
}

static void Test_Gripper_Blocked()
{
    printf("夹爪夹到物体\n");
    FakeServoPort port;
    port.servo.angle = -28;
    port.servo.obstacle_enabled = true;
    port.servo.obstacle_angle = 150;               // 物体在 15.0°
    Ds485Servo servo(&port, 1);
    Ds485Gripper gripper(&servo);
    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());
    CHECK(gripper.StartMove(255, 0, 0));
    ServoStatus st;
    CHECK(Run_Until_Done(gripper, port, &st, 3000) == GRIPPER_BLOCKED);
    CHECK(st.angle == 150);

    // 力矩太小推不动：也判为“停住”，不会一直等
    port.Reset();
    port.servo.angle = -28;
    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());
    CHECK(gripper.StartMove(255, 10, 0));          // PWM 39
    CHECK(Run_Until_Done(gripper, port, &st, 3000) == GRIPPER_BLOCKED);
}

static void Test_Gripper_Fault_Error_Stop()
{
    printf("夹爪故障/通信失败/停止\n");
    FakeServoPort port;
    port.servo.angle = -28;
    port.servo.obstacle_enabled = true;
    port.servo.obstacle_angle = 100;
    port.servo.stall_fault_ms = 50;                // 挡住 50ms 后报堵转
    Ds485Servo servo(&port, 1);
    Ds485Gripper gripper(&servo);
    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());
    CHECK(gripper.StartMove(255, 0, 0));
    ServoStatus st;
    CHECK(Run_Until_Done(gripper, port, &st, 3000) == GRIPPER_FAULT);
    CHECK(gripper.LastError().find("堵转") != std::string::npos);
    // 有保护位时拒绝再次运动（写 PWM 时短应答带保护位，报“可能已执行”）
    CHECK(!gripper.StartMove(0, 0, 0));
    CHECK(gripper.LastError().find("保护状态") != std::string::npos);
    // 保护状态下仍可锁舵停止（写入会执行，但函数报告保护状态）
    CHECK(!gripper.Stop());
    CHECK(!port.servo.moving);

    // 运动中舵机失联：连续 3 次读失败返回 ERROR
    port.Reset();
    port.servo.angle = -28;
    gripper.SetConfig(Left_Config());
    CHECK(gripper.Init());
    CHECK(gripper.StartMove(255, 0, 30));
    port.servo.no_reply = true;
    CHECK(gripper.Update(&st) == GRIPPER_RUNNING);   // 第 1 次失败不结束
    bool have = true;
    CHECK(gripper.Update(&st, &have) == GRIPPER_RUNNING);
    CHECK(!have);
    CHECK(gripper.Update(&st) == GRIPPER_ERROR);
    port.servo.no_reply = false;

    // 停止：锁舵在当前位置（上面三次超时期间舵机已到闭合位置，这里往张开方向走）
    CHECK(gripper.StartMove(0, 0, 30));
    port.SleepMs(100);
    CHECK(port.servo.moving);
    CHECK(gripper.Stop());
    CHECK(!port.servo.moving);
    CHECK(port.servo.torque_switch == DS485_SWITCH_HOLD);
}


void TestBusWorker();
void TestTrajectory();
void TestTransactions();

int main()
{
    TestTransactions();
    TestTrajectory();
    TestBusWorker();
    Test_Build_Frame();
    Test_Parser();
    Test_Servo_Probe_Status();
    Test_Servo_Move();
    Test_Servo_Write();
    Test_Protection_Config();
    Test_Protection_Recovery();
    Test_Gripper_Init();
    Test_Gripper_Mapping();
    Test_Gripper_Close_Open();
    Test_Gripper_Blocked();
    Test_Gripper_Fault_Error_Stop();

    printf("\n共 %d 项检查，失败 %d 项\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
