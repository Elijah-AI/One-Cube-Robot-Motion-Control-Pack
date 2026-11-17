/*
 * 自制夹爪（德晟舵机）离线测试。
 * 被测代码：ylf_wbc/src/motion_control_pkg/src/common/gripper/Ds485_Gripper.cpp
 * 用模拟舵机 Fake_Ds485_Bus 代替机械臂透传，不需要机器人、MarvinSDK 和 ROS。
 *
 * 运行：./test_ds485_gripper        全部通过返回 0，否则返回 1
 */
#include <stdio.h>
#include <string.h>

#include "Ds485_Gripper.h"
#include "fake_ds485_servo.h"


// ======================= 极简断言 =======================

static int g_passed = 0;
static int g_failed = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (cond) {                                                        \
            g_passed++;                                                    \
        } else {                                                           \
            g_failed++;                                                    \
            printf("    [失败] %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
        }                                                                  \
    } while (0)

#define CHECK_EQ(a, b)                                                     \
    do {                                                                   \
        long _va = (long)(a);                                              \
        long _vb = (long)(b);                                              \
        if (_va == _vb) {                                                  \
            g_passed++;                                                    \
        } else {                                                           \
            g_failed++;                                                    \
            printf("    [失败] %s:%d  %s == %s  (实际 %ld, 期望 %ld)\n",   \
                   __FILE__, __LINE__, #a, #b, _va, _vb);                  \
        }                                                                  \
    } while (0)

static void Begin(const char *name)
{
    printf("[测试] %s\n", name);
}


// ======================= 辅助函数 =======================

// 按运控节点的方式配置一侧夹爪：通道 2，张开 0°，闭合 90°
static void Setup_Side(Ds485_Gripper &gripper, int side, uint32_t channel, uint8_t id)
{
    gripper.Ds485_Config(side, channel, id);
    gripper.Ds485_Set_Angle_Range(side, 0, 900);      // 0.0° ~ 90.0°
    gripper.Ds485_Set_Speed(side, 30, 90);            // 默认 30°/s，speed=255 对应 90°/s
    gripper.Ds485_Set_Detect(side, 10, 3, 10);        // 容差 1.0°，不动带 0.3°，10 次
    gripper.Ds485_Set_Bus_Timing(side, 10, 300);
}

// 模拟运控节点的轮询：每 20ms 读一次状态并判断，返回最终判断结果
static int Run_Until_Done(Ds485_Gripper &gripper, Fake_Ds485_Bus &bus, int side,
                          Ds485_status *status, int max_loops)
{
    int result = DS485_MOVE_RUNNING;
    for (int i = 0; i < max_loops; i++)
    {
        bus.Sleep_ms(20);
        if (!gripper.Ds485_Status(side, status))
        {
            printf("    读状态失败：%s\n", gripper.Ds485_Last_Error(side));
            return -1;
        }
        result = gripper.Ds485_Watch_Update(side, *status);
        if (result != DS485_MOVE_RUNNING)
        {
            return result;
        }
    }
    return result;
}

// 初始化 + 读初始状态 + 下发 Move_Abs + 启动判断
static bool Start_Move(Ds485_Gripper &gripper, int side, uint8_t position, uint8_t torque, uint8_t speed)
{
    Ds485_status status;
    memset(&status, 0, sizeof(status));
    if (!gripper.Ds485_Init(side))
    {
        printf("    初始化失败：%s\n", gripper.Ds485_Last_Error(side));
        return false;
    }
    if (!gripper.Ds485_Status(side, &status))
    {
        printf("    读状态失败：%s\n", gripper.Ds485_Last_Error(side));
        return false;
    }
    if (!gripper.Ds485_Move_Abs(side, position, torque, speed))
    {
        printf("    下发失败：%s\n", gripper.Ds485_Last_Error(side));
        return false;
    }
    gripper.Ds485_Watch_Start(side, gripper.Ds485_Target_Angle(side), status.angle);
    return true;
}


// ======================= 测试用例 =======================

// 1. 帧编码与校验，对照 README 中实测报文
static void Test_Frame()
{
    Begin("帧编码与校验和");
    uint8_t out[256];

    // 读 ID1 的 0x01：F9 FF 01 03 02 01 F8
    int n = Ds485_Gripper::Ds485_Build_Frame(1, DS485_CMD_READ, 0x01, 0, 0, out);
    CHECK_EQ(n, 7);
    CHECK_EQ(out[0], 0xF9);
    CHECK_EQ(out[1], 0xFF);
    CHECK_EQ(out[2], 0x01);
    CHECK_EQ(out[3], 0x03);
    CHECK_EQ(out[4], 0x02);
    CHECK_EQ(out[5], 0x01);
    CHECK_EQ(out[6], 0xF8);

    // 写 0x66：90.0°（900=0x0384），30°/s（0x001E）
    uint8_t payload[4] = {0x84, 0x03, 0x1E, 0x00};
    n = Ds485_Gripper::Ds485_Build_Frame(1, DS485_CMD_WRITE, 0x66, payload, 4, out);
    CHECK_EQ(n, 11);
    CHECK_EQ(out[3], 7);
    uint32_t sum = 0;
    for (int i = 2; i < 10; i++)
    {
        sum += out[i];
    }
    CHECK_EQ(out[10], (~sum) & 0xFF);

    // 参数过长被拒绝
    uint8_t big[300];
    memset(big, 0, sizeof(big));
    CHECK_EQ(Ds485_Gripper::Ds485_Build_Frame(1, DS485_CMD_WRITE, 0x66, big, 250, out), 0);
}

// 2. 位置 0~255 与角度互换
static void Test_Mapping()
{
    Begin("位置/角度换算");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);

    gripper.Ds485_Set_Angle_Range(DS485_SIDE_LEFT, 0, 900);
    CHECK_EQ(gripper.Ds485_Position_To_Angle(DS485_SIDE_LEFT, 0), 0);
    CHECK_EQ(gripper.Ds485_Position_To_Angle(DS485_SIDE_LEFT, 255), 900);
    CHECK_EQ(gripper.Ds485_Position_To_Angle(DS485_SIDE_LEFT, 128), 451);
    CHECK_EQ(gripper.Ds485_Angle_To_Position(DS485_SIDE_LEFT, 450), 128);
    CHECK_EQ(gripper.Ds485_Angle_To_Position(DS485_SIDE_LEFT, -100), 0);    // 超出截断
    CHECK_EQ(gripper.Ds485_Angle_To_Position(DS485_SIDE_LEFT, 1200), 255);

    // 反向安装：张开 90°，闭合 0°
    gripper.Ds485_Set_Angle_Range(DS485_SIDE_RIGHT, 900, 0);
    CHECK_EQ(gripper.Ds485_Position_To_Angle(DS485_SIDE_RIGHT, 0), 900);
    CHECK_EQ(gripper.Ds485_Position_To_Angle(DS485_SIDE_RIGHT, 255), 0);
    CHECK_EQ(gripper.Ds485_Angle_To_Position(DS485_SIDE_RIGHT, 900), 0);
    CHECK_EQ(gripper.Ds485_Angle_To_Position(DS485_SIDE_RIGHT, 0), 255);

    // 负角度范围
    gripper.Ds485_Set_Angle_Range(DS485_SIDE_LEFT, -450, 450);
    CHECK_EQ(gripper.Ds485_Position_To_Angle(DS485_SIDE_LEFT, 0), -450);
    CHECK_EQ(gripper.Ds485_Angle_To_Position(DS485_SIDE_LEFT, 0), 128);
}

// 3. 初始化检查
static void Test_Init_Checks()
{
    Begin("初始化检查：未标定、超限位、电机模式");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);

    // 未标定
    gripper.Ds485_Config(DS485_SIDE_LEFT, 2, 1);
    CHECK(!gripper.Ds485_Init(DS485_SIDE_LEFT));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "未标定") != 0);
    CHECK_EQ(bus.servo[0].frames_ok, 0);   // 未标定时不访问总线

    // 标定范围超出舵机限位 ±180°
    gripper.Ds485_Set_Angle_Range(DS485_SIDE_LEFT, 0, 2000);
    CHECK(!gripper.Ds485_Init(DS485_SIDE_LEFT));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "限位") != 0);

    // 电机模式
    gripper.Ds485_Set_Angle_Range(DS485_SIDE_LEFT, 0, 900);
    bus.servo[0].mode = 2;
    CHECK(!gripper.Ds485_Init(DS485_SIDE_LEFT));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "模式") != 0);

    // 正常
    bus.servo[0].mode = 1;
    CHECK(gripper.Ds485_Init(DS485_SIDE_LEFT));

    // 未初始化直接运动被拒绝
    Ds485_Gripper fresh(&bus);
    Setup_Side(fresh, DS485_SIDE_LEFT, 2, 1);
    CHECK(!fresh.Ds485_Move_Abs(DS485_SIDE_LEFT, 255, 0, 0));
    CHECK(strstr(fresh.Ds485_Last_Error(DS485_SIDE_LEFT), "未初始化") != 0);
}

// 4. 正常闭合到位（无物体）
static void Test_Close_Arrive()
{
    Begin("闭合到位：自动打开扭矩、写 PWM 上限、按速度运动");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);

    // torque=100 -> PWM 392；speed=255 -> 90°/s
    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 255, 100, 255));
    CHECK_EQ(bus.servo[0].pwm_limit, 392);
    CHECK_EQ(bus.servo[0].torque_switch, DS485_SWITCH_HOLD);
    CHECK_EQ(bus.servo[0].target, 900);
    CHECK_EQ(bus.servo[0].move_speed, 90);
    CHECK_EQ(bus.servo[0].last_channel, 2);

    Ds485_status status;
    memset(&status, 0, sizeof(status));
    long start = bus.time_ms;
    int result = Run_Until_Done(gripper, bus, DS485_SIDE_LEFT, &status, 500);
    CHECK_EQ(result, DS485_MOVE_ARRIVED);
    CHECK_EQ(status.angle, 900);
    CHECK_EQ(status.position, 255);
    CHECK(!status.moving);
    // 90° / 90°/s ≈ 1 秒（模拟时间，包含通信等待）
    CHECK(bus.time_ms - start >= 1000);
    printf("    模拟用时 %ld ms\n", bus.time_ms - start);

    // 再张开
    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 0, 0, 0));
    CHECK_EQ(bus.servo[0].pwm_limit, 1000);       // torque=0 恢复默认 PWM 上限
    CHECK_EQ(bus.servo[0].move_speed, 30);        // speed=0 使用默认速度
    result = Run_Until_Done(gripper, bus, DS485_SIDE_LEFT, &status, 500);
    CHECK_EQ(result, DS485_MOVE_ARRIVED);
    CHECK_EQ(status.position, 0);
}

// 5. 闭合时夹到物体
static void Test_Close_Blocked()
{
    Begin("闭合夹到物体：未到目标停止 -> BLOCKED");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    bus.servo[0].obstacle_enabled = true;
    bus.servo[0].obstacle_angle = 600;   // 物体在 60°

    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 255, 80, 128));
    Ds485_status status;
    memset(&status, 0, sizeof(status));
    int result = Run_Until_Done(gripper, bus, DS485_SIDE_LEFT, &status, 500);
    CHECK_EQ(result, DS485_MOVE_BLOCKED);
    CHECK_EQ(status.angle, 600);
    CHECK_EQ(status.position, 170);        // 60/90*255
    CHECK(status.moving);                  // 仍在用力夹持
    CHECK(status.pwm > 0);
    printf("    夹持位置 %d，PWM %d\n", status.position, status.pwm);
}

// 6. 夹持过久触发堵转保护
static void Test_Stall_Fault()
{
    Begin("运动中出现堵转保护 -> FAULT");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    gripper.Ds485_Set_Detect(DS485_SIDE_LEFT, 10, 3, 1000);   // 不让 BLOCKED 先触发
    bus.servo[0].obstacle_enabled = true;
    bus.servo[0].obstacle_angle = 300;
    bus.servo[0].stall_fault_ms = 200;

    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 255, 0, 255));
    Ds485_status status;
    memset(&status, 0, sizeof(status));
    int result = Run_Until_Done(gripper, bus, DS485_SIDE_LEFT, &status, 500);
    CHECK_EQ(result, DS485_MOVE_FAULT);
    CHECK(status.err_stall);
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "堵转") != 0);
    printf("    错误信息：%s\n", gripper.Ds485_Last_Error(DS485_SIDE_LEFT));

    // 有保护位时拒绝下发新运动
    CHECK(!gripper.Ds485_Move_Abs(DS485_SIDE_LEFT, 0, 0, 0));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "状态异常") != 0);
}

// 7. 写应答关闭时仍可工作（靠读回确认）
static void Test_No_Write_Ack()
{
    Begin("舵机关闭写应答：靠读回确认");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    bus.servo[0].sys_config = 0x00;

    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 128, 255, 255));
    CHECK_EQ(bus.servo[0].pwm_limit, 1000);
    Ds485_status status;
    memset(&status, 0, sizeof(status));
    CHECK_EQ(Run_Until_Done(gripper, bus, DS485_SIDE_LEFT, &status, 500), DS485_MOVE_ARRIVED);
    CHECK_EQ(status.angle, 451);

    // 写入被舵机拒绝（无应答），读回不一致应报错
    bus.servo[0].reject_switch_write = true;
    CHECK(!gripper.Ds485_Release(DS485_SIDE_LEFT));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "读回不一致") != 0);
}

// 8. 写入被拒绝（有写应答）
static void Test_Write_Rejected()
{
    Begin("写入被拒绝：短应答 0x40 -> 失败并给出原因");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    bus.servo[0].reject_switch_write = true;

    CHECK(gripper.Ds485_Init(DS485_SIDE_LEFT));
    CHECK(!gripper.Ds485_Move_Abs(DS485_SIDE_LEFT, 255, 0, 0));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "0x64") != 0);
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "命令执行失败") != 0);
    CHECK(!bus.servo[0].moving);
    printf("    错误信息：%s\n", gripper.Ds485_Last_Error(DS485_SIDE_LEFT));
}

// 9. 通信干扰：噪声、坏校验、分包
static void Test_Noise()
{
    Begin("通信干扰：噪声、坏校验、分包都能正确解析");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    bus.servo[0].angle = 123;
    bus.servo[0].target = 123;

    int32_t angle = 0;
    bus.servo[0].add_noise = true;
    CHECK(gripper.Ds485_Read_I32(DS485_SIDE_LEFT, DS485_REG_ANGLE, &angle));
    CHECK_EQ(angle, 123);

    bus.servo[0].add_noise = false;
    bus.servo[0].corrupt_next_reply = true;
    angle = 0;
    CHECK(gripper.Ds485_Read_I32(DS485_SIDE_LEFT, DS485_REG_ANGLE, &angle));
    CHECK_EQ(angle, 123);

    bus.servo[0].split_reply = true;
    angle = 0;
    CHECK(gripper.Ds485_Read_I32(DS485_SIDE_LEFT, DS485_REG_ANGLE, &angle));
    CHECK_EQ(angle, 123);

    // 三种干扰同时存在，完整走一次闭合
    bus.servo[0].add_noise = true;
    bus.servo[0].split_reply = true;
    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 255, 0, 255));
    Ds485_status status;
    memset(&status, 0, sizeof(status));
    CHECK_EQ(Run_Until_Done(gripper, bus, DS485_SIDE_LEFT, &status, 500), DS485_MOVE_ARRIVED);
    CHECK_EQ(status.angle, 900);
}

// 10. 不应答超时
static void Test_Timeout()
{
    Begin("舵机不应答：读取超时返回失败");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    bus.servo[0].no_reply = true;

    long start = bus.time_ms;
    Ds485_status status;
    memset(&status, 0, sizeof(status));
    CHECK(!gripper.Ds485_Status(DS485_SIDE_LEFT, &status));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "超时") != 0);
    long used = bus.time_ms - start;
    CHECK(used >= 300 && used <= 400);   // 约等于 timeout_ms
    printf("    超时用时 %ld ms，错误信息：%s\n", used, gripper.Ds485_Last_Error(DS485_SIDE_LEFT));

    // ID 不对也是超时
    bus.servo[0].no_reply = false;
    gripper.Ds485_Config(DS485_SIDE_LEFT, 2, 5);
    CHECK(!gripper.Ds485_Status(DS485_SIDE_LEFT, &status));
}

// 11. 配置函数：扭矩、电流、速度上限，开关
static void Test_Config_Functions()
{
    Begin("配置函数：扭矩/电流/速度上限、锁舵/刹车/释放、健康数据");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);

    CHECK(gripper.Ds485_Set_Torque(DS485_SIDE_LEFT, 300));
    CHECK_EQ(bus.servo[0].pwm_limit, 300);
    int writes = bus.servo[0].write_count;
    CHECK(!gripper.Ds485_Set_Torque(DS485_SIDE_LEFT, 1001));   // 超范围不发送
    CHECK_EQ(bus.servo[0].write_count, writes);

    CHECK(gripper.Ds485_Set_Current_Limit(DS485_SIDE_LEFT, 1500));
    CHECK_EQ(bus.servo[0].current_limit, 1500);
    CHECK(gripper.Ds485_Set_Speed_Limit(DS485_SIDE_LEFT, 120));
    CHECK_EQ(bus.servo[0].speed_limit, 120);
    CHECK(!gripper.Ds485_Set_Speed_Limit(DS485_SIDE_LEFT, 0));

    CHECK(gripper.Ds485_Hold(DS485_SIDE_LEFT));
    CHECK_EQ(bus.servo[0].torque_switch, 2);
    CHECK(gripper.Ds485_Brake(DS485_SIDE_LEFT));
    CHECK_EQ(bus.servo[0].torque_switch, 1);
    CHECK(gripper.Ds485_Release(DS485_SIDE_LEFT));
    CHECK_EQ(bus.servo[0].torque_switch, 0);

    Ds485_status status;
    memset(&status, 0, sizeof(status));
    CHECK(gripper.Ds485_Read_Health(DS485_SIDE_LEFT, &status));
    CHECK_EQ(status.temperature, 28);
    CHECK_EQ(status.voltage, 23941);
    CHECK_EQ(status.mode, 1);
    CHECK_EQ(status.torque_switch, 0);
    CHECK_EQ(status.pwm_limit, 300);

    // 停止：运动中锁舵，停在当前位置
    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 255, 0, 255));
    bus.Sleep_ms(300);
    CHECK(bus.servo[0].moving);
    CHECK(gripper.Ds485_Hold(DS485_SIDE_LEFT));
    CHECK(!bus.servo[0].moving);
    int32_t stop_angle = bus.servo[0].angle;
    bus.Sleep_ms(500);
    CHECK_EQ(bus.servo[0].angle, stop_angle);
    CHECK(stop_angle > 0 && stop_angle < 900);
}

// 12. 左右独立：不同通道、不同 ID，同时运动
static void Test_Both_Sides()
{
    Begin("左右夹爪独立透传：同时闭合，一侧夹到物体");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    Setup_Side(gripper, DS485_SIDE_RIGHT, 3, 2);
    bus.servo[1].id = 2;
    bus.servo[1].sys_config = 0x00;          // 右侧关闭写应答
    bus.servo[1].obstacle_enabled = true;
    bus.servo[1].obstacle_angle = 450;

    CHECK(Start_Move(gripper, DS485_SIDE_LEFT, 255, 0, 255));
    CHECK(Start_Move(gripper, DS485_SIDE_RIGHT, 255, 0, 255));
    CHECK_EQ(bus.servo[0].last_channel, 2);
    CHECK_EQ(bus.servo[1].last_channel, 3);

    int result[2] = {DS485_MOVE_RUNNING, DS485_MOVE_RUNNING};
    Ds485_status status[2];
    memset(status, 0, sizeof(status));
    for (int loop = 0; loop < 500; loop++)
    {
        bus.Sleep_ms(20);
        for (int side = 0; side < 2; side++)
        {
            if (result[side] != DS485_MOVE_RUNNING)
            {
                continue;
            }
            CHECK(gripper.Ds485_Status(side, &status[side]));
            result[side] = gripper.Ds485_Watch_Update(side, status[side]);
        }
        if (result[0] != DS485_MOVE_RUNNING && result[1] != DS485_MOVE_RUNNING)
        {
            break;
        }
    }
    CHECK_EQ(result[0], DS485_MOVE_ARRIVED);
    CHECK_EQ(result[1], DS485_MOVE_BLOCKED);
    CHECK_EQ(status[0].angle, 900);
    CHECK_EQ(status[1].angle, 450);
    CHECK_EQ(bus.servo[0].frames_bad, 0);
    CHECK_EQ(bus.servo[1].frames_bad, 0);
}

// 13. 判断逻辑：转动位一直为 1 但已停在目标附近 -> 到位
static void Test_Watch_Logic()
{
    Begin("判断逻辑：边界情况");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    Ds485_status s;
    memset(&s, 0, sizeof(s));

    // 转动位一直为 1，但角度已在容差内且不再变化
    gripper.Ds485_Watch_Start(DS485_SIDE_LEFT, 900, 0);
    s.moving = true;
    s.angle = 895;
    int result = DS485_MOVE_RUNNING;
    for (int i = 0; i < 20 && result == DS485_MOVE_RUNNING; i++)
    {
        result = gripper.Ds485_Watch_Update(DS485_SIDE_LEFT, s);
    }
    CHECK_EQ(result, DS485_MOVE_ARRIVED);

    // 停止且在容差内：连续 2 次即到位
    gripper.Ds485_Watch_Start(DS485_SIDE_LEFT, 900, 0);
    s.moving = false;
    s.angle = 905;
    CHECK_EQ(gripper.Ds485_Watch_Update(DS485_SIDE_LEFT, s), DS485_MOVE_RUNNING);
    CHECK_EQ(gripper.Ds485_Watch_Update(DS485_SIDE_LEFT, s), DS485_MOVE_ARRIVED);

    // 一开始就夹着物体（完全不动）：判定 BLOCKED
    gripper.Ds485_Watch_Start(DS485_SIDE_LEFT, 900, 500);
    s.moving = true;
    s.angle = 500;
    result = DS485_MOVE_RUNNING;
    int count = 0;
    while (result == DS485_MOVE_RUNNING && count < 50)
    {
        result = gripper.Ds485_Watch_Update(DS485_SIDE_LEFT, s);
        count++;
    }
    CHECK_EQ(result, DS485_MOVE_BLOCKED);
    CHECK_EQ(count, 10);

    // 故障位优先
    s.state_raw = 0x82;   // 转动中 + 过流
    CHECK_EQ(gripper.Ds485_Watch_Update(DS485_SIDE_LEFT, s), DS485_MOVE_FAULT);
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "过流") != 0);

    // 故障文字
    char text[120];
    Ds485_Gripper::Ds485_Fault_Text(0x00, text, sizeof(text));
    CHECK(strcmp(text, "无故障") == 0);
    Ds485_Gripper::Ds485_Fault_Text(0x48, text, sizeof(text));
    CHECK(strcmp(text, "堵转/过载保护、命令执行失败") == 0);
}

// 14. 超出标定范围的角度被拒绝
static void Test_Range_Guard()
{
    Begin("角度保护：超出标定范围不下发");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, DS485_SIDE_LEFT, 2, 1);
    CHECK(gripper.Ds485_Init(DS485_SIDE_LEFT));
    int writes = bus.servo[0].write_count;
    CHECK(!gripper.Ds485_Move_Angle(DS485_SIDE_LEFT, 1000, 30));
    CHECK(strstr(gripper.Ds485_Last_Error(DS485_SIDE_LEFT), "标定范围") != 0);
    CHECK(!gripper.Ds485_Move_Angle(DS485_SIDE_LEFT, -10, 30));
    CHECK(!gripper.Ds485_Move_Angle(DS485_SIDE_LEFT, 450, 0));
    CHECK_EQ(bus.servo[0].write_count, writes);
    CHECK(gripper.Ds485_Move_Angle(DS485_SIDE_LEFT, 450, 30));
}


static void Test_Defaults_And_Speed_Limit()
{
    Begin("零力矩/速度默认值与舵机速度上限");
    Fake_Ds485_Bus bus;
    Ds485_Gripper gripper(&bus);
    Setup_Side(gripper, 0, 2, 1);
    bus.servo[0].speed_limit = 120;
    gripper.Ds485_Set_Speed(0, 0, 0);
    CHECK(gripper.Ds485_Init(0));
    CHECK_EQ(bus.servo[0].write_count, 0);
    CHECK(gripper.Ds485_Move_Abs(0, 255, 0, 0));
    CHECK_EQ(bus.servo[0].pwm_limit, 1000);
    CHECK_EQ(bus.servo[0].move_speed, 120);
    CHECK_EQ(bus.servo[0].speed_limit, 120);
    CHECK(gripper.Ds485_Hold(0));
    gripper.Ds485_Set_Default_PWM(0, 320);
    CHECK(gripper.Ds485_Move_Abs(0, 0, 0, 128));
    CHECK_EQ(bus.servo[0].pwm_limit, 320);
    CHECK_EQ(bus.servo[0].move_speed, 60);
    CHECK(gripper.Ds485_Hold(0));
    gripper.Ds485_Set_Speed(0, 600, 900);
    CHECK(gripper.Ds485_Init(0));
    CHECK_EQ(gripper.Ds485_Get_Config(0).max_speed, 120);
    CHECK_EQ(gripper.Ds485_Get_Config(0).default_speed, 120);
    const int writes = bus.servo[0].write_count;
    CHECK(!gripper.Ds485_Move_Angle(0, 0, 121));
    CHECK_EQ(bus.servo[0].write_count, writes);
    bus.servo[0].speed_limit = 0;
    CHECK(!gripper.Ds485_Init(0));
    CHECK(!gripper.Ds485_Move_Abs(0, 255, 0, 0));
    CHECK_EQ(bus.servo[0].write_count, writes);
    bus.servo[0].speed_limit = 120;
    gripper.Ds485_Set_Default_PWM(0, 0);
    CHECK(!gripper.Ds485_Init(0));
    CHECK_EQ(bus.servo[0].write_count, writes);
}

int main()
{
    Test_Defaults_And_Speed_Limit();
    printf("==== 自制夹爪（德晟舵机）离线测试 ====\n");
    Test_Frame();
    Test_Mapping();
    Test_Init_Checks();
    Test_Close_Arrive();
    Test_Close_Blocked();
    Test_Stall_Fault();
    Test_No_Write_Ack();
    Test_Write_Rejected();
    Test_Noise();
    Test_Timeout();
    Test_Config_Functions();
    Test_Both_Sides();
    Test_Watch_Logic();
    Test_Range_Guard();

    printf("==== 结果：通过 %d 项，失败 %d 项 ====\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
