#ifndef __MODBUS_JODELL_H
#define __MODBUS_JODELL_H

#include <cstdint>

#define MAX_OPERATION_REG_NUM 100
#define OPERATION_REG_NUM 3
#define STATUS_REG_NUM    4


enum ChannelIndex
{
    left=0,
    right,
};

struct Jodell_status
{
    // --- 故障标志位 ---
    bool err_active;             // 激活故障
    bool err_control;            // 控制故障
    bool err_comm;               // 通信故障
    bool err_over_current;       // 过流故障
    bool err_voltage;            // 电压异常故障
    bool err_enable;             // 使能故障
    bool err_over_temp;          // 过温故障 
    bool err_self;               // 产品自身故障

    // --- 实时状态数据 ---
    uint8_t  position;           // 当前位置 (0-255)
    uint8_t  speed;              // 当前速度 (0-255)
    uint8_t  torque;             // 当前扭矩 (0-255)
    uint8_t  busvoltage;         // 母线电压 (0-255, 例如代表 0-25.5V 或需缩放)
    int8_t   temperature;        // 当前温度 (-128 ~ 127 °C)

    // ----夹爪状态数据-----
    uint8_t  status;             // 夹爪状态 0：复位中， 1：处于使能状态
    uint8_t  mode;               // 工作模式 0：参数控制模式，1：无参数控制模式
    uint8_t  gto;                // 动作状态 0：停止，1：前往目标位置
    uint8_t  state;              // 夹爪的当前状态和动作 0:电动夹爪处于复位或者巡检状态，如果已激活，需要查看故障码, 1:正在激活, 2:未使用，保留, 3:激活完成
    int8_t   obj;                // 目标检测 0:手指正向指定位置移动，未检测到物体,1:手指在张开达到指定位置时，由于接触到物体已经停止，表示为内撑模式已接触到物体
    //2:手指在闭合到达指定位置时，由于接触到物体已经停止，表示为外夹模式已接触到物体, 3:手指已到达指定的位置，但没有检测到对象，或者物体已丢失或未夹住脱落。
};


class Jodell_Control
{
    public:
    Jodell_Control();
    ~Jodell_Control();

    /**
     * @brief 控制夹爪按照设定的力矩与速度到底指定的位置 
     * @param position  目标位置 0-255, 0表示全部打开，255表示全部闭合。中间按照现行进行开合
     * @param torque    力矩 0~255
     * @param speed     速度 0-255
     * @return true: 成功, false: 失败
     */
    bool Jodell_Move_Abs_A(uint8_t position, uint8_t torque, uint8_t speed);
    bool Jodell_Move_Abs_B(uint8_t position, uint8_t torque, uint8_t speed);

    /**
     * @brief 控制夹爪按照设定的力矩与速度到底指定的位置
     * @param rt_status     设备状态信息
     * @return true: 成功, false: 失败
     */
    bool Jodell_Status_A(Jodell_status *rt_status);
    bool Jodell_Status_B(Jodell_status *rt_status);

    /**
     * @brief 配置夹爪通信方式与通道
     * @param rt_status     设备状态信息
     * @return true: 成功, false: 失败
     */
    void Jodell_modbus_config_A(uint32_t ch_value, uint8_t id);
    void Jodell_modbus_config_B(uint32_t ch_value, uint8_t id);


    /**
     * @brief 按照modbus协议组单个寄存器写入包通过机械臂透传发送给夹爪 (功能码 0x06)
     * @param addr     待写入的寄存器地址
     * @param value     待写入的寄存器的值
     * @return true: 成功, false: 失败
     */
    bool Jodell_modbus_register_write_A(uint16_t addr, uint16_t value);

    /**
     * @brief 按照modbus协议组多个寄存器写入包通过机械臂透传发送给夹爪 (功能码 0x10)
     * @param start_addr     待写入的起始寄存器地址
     * @param cnt            待写入的寄存器的数量
     * @param value          待写入的寄存器的值
     * @return true: 成功, false: 失败
     */
    bool Jodell_modbus_register_writes_A(uint16_t start_addr, uint16_t cnt, uint16_t value[]);

    /**
     * @brief 按照modbus协议组寄存器读取包通过机械臂透传发送给夹爪 (功能码 0x03)
     * @param start_addr     待读取的起始寄存器地址
     * @param cnt            待读取的寄存器的数量
     * @param data           读取的寄存器的值
     * @return 读取到的数据长度
     */
    uint8_t Jodell_modbus_register_read_A(uint16_t start_addr, uint16_t cnt, uint8_t *data);
    bool Jodell_modbus_register_write_B(uint16_t addr, uint16_t value);
    bool Jodell_modbus_register_writes_B(uint16_t start_addr, uint16_t cnt, uint16_t value[]);
    uint8_t Jodell_modbus_register_read_B(uint16_t start_addr, uint16_t cnt, uint8_t *data);
    uint16_t calculateModbusCRC16(const uint8_t* data, uint32_t length);

    protected:
    

    private:
    uint32_t ch[2];        //通道编号 1：CAN， 2：COM1， 3：COM2
    uint8_t slaveid[2];    //节点设备ID
    

};

















#endif
