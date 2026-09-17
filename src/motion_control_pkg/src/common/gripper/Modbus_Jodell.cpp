#include "Modbus_Jodell.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include "MarvinSDK.h" 
#include <string.h>
#include "FxRobot.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <cstdlib>



const uint16_t register_move_control = 0x3e8;

const uint16_t register_status = 0x7d0;
const uint16_t register_status_num = 4; 

const uint8_t func_code_write_single_reg = 0x06;
const uint8_t func_code_write_multi_reg = 0x10;
const uint8_t func_code_read_reg = 0x03;

const uint16_t move_abs_no_param_reg_value = 0x09;


// Modbus CRC16 查找表 (基于多项式 0xA001)
static const uint16_t crc16_table[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};


Jodell_Control::Jodell_Control()
{

    ch[left] = 0;
    ch[right] = 0;
    slaveid[left] = 0;
    slaveid[right] = 0;
}


Jodell_Control::~Jodell_Control()
{

}



bool Jodell_Control::Jodell_Move_Abs_A(uint8_t position, uint8_t torque, uint8_t speed)
{
    uint16_t value[OPERATION_REG_NUM] = {0,0,0};
    value[0] = move_abs_no_param_reg_value;
    value[1] = (position<<8);
    value[2] = ((torque<<8) | speed);
    return Jodell_modbus_register_writes_A(register_move_control, OPERATION_REG_NUM, value);
}


bool Jodell_Control::Jodell_Move_Abs_B(uint8_t position, uint8_t torque, uint8_t speed)
{
    uint16_t value[OPERATION_REG_NUM] = {0,0,0};
    value[0] = move_abs_no_param_reg_value;
    value[1] = (position<<8);
    value[2] = ((torque<<8) | speed);
    return Jodell_modbus_register_writes_B(register_move_control, OPERATION_REG_NUM, value);

}

bool Jodell_Control::Jodell_Status_A(Jodell_status *rt_status)
{
    uint8_t hex_str[256] = {0};
    uint8_t size = Jodell_modbus_register_read_A(register_status, STATUS_REG_NUM, hex_str);
    if (size > 0)
    {
        rt_status->status = (hex_str[4]&0x01);
        rt_status->mode   = ((hex_str[4]&0x04) >> 2);
        rt_status->gto    = ((hex_str[4]&0x08) >> 3);
        rt_status->state  = ((hex_str[4]&0x30) >> 4);
        rt_status->obj    = ((hex_str[4]&0xC0) >> 6);

        rt_status->position     = hex_str[5];
        rt_status->err_active   = ((hex_str[6]&0x01) >> 0);
        rt_status->err_control  = ((hex_str[6]&0x02) >> 1);
        rt_status->err_comm     = ((hex_str[6]&0x04) >> 2);
        rt_status->err_over_current   = ((hex_str[6]&0x08) >> 3);
        rt_status->err_voltage   = ((hex_str[6]&0x10) >> 4);
        rt_status->err_enable   = ((hex_str[6]&0x20) >> 5);
        rt_status->err_over_temp = ((hex_str[6]&0x40) >> 6);
        rt_status->err_self = ((hex_str[6]&0x80) >> 7);
        rt_status->torque = hex_str[7];
        rt_status->speed = hex_str[8];
        rt_status->temperature = hex_str[9];
        rt_status->busvoltage = hex_str[10];
        return true;

    }
    


    
    return false;
}


bool Jodell_Control::Jodell_Status_B(Jodell_status *rt_status)
{
    uint8_t hex_str[256] = {0};
    uint8_t size = Jodell_modbus_register_read_B(register_status, STATUS_REG_NUM, hex_str);
    if (size > 0)
    {
        rt_status->status = (hex_str[4]&0x01);
        rt_status->mode   = ((hex_str[4]&0x04) >> 2);
        rt_status->gto    = ((hex_str[4]&0x08) >> 3);
        rt_status->state  = ((hex_str[4]&0x30) >> 4);
        rt_status->obj    = ((hex_str[4]&0xC0) >> 6);

        rt_status->position     = hex_str[5];
        rt_status->err_active   = ((hex_str[6]&0x01) >> 0);
        rt_status->err_control  = ((hex_str[6]&0x02) >> 1);
        rt_status->err_comm     = ((hex_str[6]&0x04) >> 2);
        rt_status->err_over_current   = ((hex_str[6]&0x08) >> 3);
        rt_status->err_voltage   = ((hex_str[6]&0x10) >> 4);
        rt_status->err_enable   = ((hex_str[6]&0x20) >> 5);
        rt_status->err_over_temp = ((hex_str[6]&0x40) >> 6);
        rt_status->err_self = ((hex_str[6]&0x80) >> 7);
        rt_status->torque = hex_str[7];
        rt_status->speed = hex_str[8];
        rt_status->temperature = hex_str[9];
        rt_status->busvoltage = hex_str[10];
        return true;

    }
    


    
    return false;
}

bool Jodell_Control::Jodell_modbus_register_write_A(uint16_t addr, uint16_t value)
{
    uint8_t hex_str[256] = {0};

    hex_str[0] = slaveid[left];
    hex_str[1] = func_code_write_single_reg;
    hex_str[2] = ((addr>>8)&0xff);
    hex_str[3] = (addr &0xff);
    hex_str[4] = ((value>>8)&0xff);
    hex_str[5] = (value &0xff);
    uint16_t crc = calculateModbusCRC16(hex_str, 6);
    memcpy(&hex_str[6], &crc, 2);
    OnClearChDataA();
    usleep(50000);
    int data_size = 8;
    bool result = OnSetChDataA(hex_str, data_size, ch[left]);

    return result;
}


bool Jodell_Control::Jodell_modbus_register_write_B(uint16_t addr, uint16_t value)
{
    uint8_t hex_str[256] = {0};

    hex_str[0] = slaveid[right];
    hex_str[1] = func_code_write_single_reg;
    hex_str[2] = ((addr>>8)&0xff);
    hex_str[3] = (addr &0xff);
    hex_str[4] = ((value>>8)&0xff);
    hex_str[5] = (value &0xff);
    uint16_t crc = calculateModbusCRC16(hex_str, 6);
    memcpy(&hex_str[6], &crc, 2);
    OnClearChDataB();
    usleep(50000);
    int data_size = 8;
    bool result = OnSetChDataB(hex_str, data_size, ch[right]);

    return result;
}


bool Jodell_Control::Jodell_modbus_register_writes_A(uint16_t start_addr, uint16_t cnt, uint16_t value[])
{
    uint8_t hex_str[256] = {0};

    if (cnt > MAX_OPERATION_REG_NUM)
    {
        printf("cnt invalid\n");
        return false;
    }


    hex_str[0] = slaveid[left];
    hex_str[1] = func_code_write_multi_reg;
    hex_str[2] = ((start_addr>>8)&0xff);
    hex_str[3] = (start_addr &0xff);
    hex_str[4] = ((cnt>>8)&0xff);
    hex_str[5] = (cnt &0xff);
    hex_str[6] = cnt*2;
    uint8_t i;
    for (i = 0; i < cnt; i++)
    {
        
        hex_str[7+2*i] = ((value[i]>>8)&0xff);
        hex_str[8+2*i] = (value[i] &0xff);
    }
    uint16_t crc = calculateModbusCRC16(hex_str, 7+2*i);
    memcpy(&hex_str[7+2*i], &crc, 2);
    long data_size = 9 + 2*i;

    usleep(50000);
    //发送数据前，先清缓存
    OnClearChDataA();
    usleep(50000);
    return OnSetChDataA(hex_str, data_size, ch[left]);

}


bool Jodell_Control::Jodell_modbus_register_writes_B(uint16_t start_addr, uint16_t cnt, uint16_t value[])
{
    uint8_t hex_str[256] = {0};

    if (cnt > MAX_OPERATION_REG_NUM)
    {
        printf("cnt invalid\n");
        return false;
    }


    hex_str[0] = slaveid[right];
    hex_str[1] = func_code_write_multi_reg;
    hex_str[2] = ((start_addr>>8)&0xff);
    hex_str[3] = (start_addr &0xff);
    hex_str[4] = ((cnt>>8)&0xff);
    hex_str[5] = (cnt &0xff);
    hex_str[6] = cnt*2;
    uint8_t i;
    for (i = 0; i < cnt; i++)
    {
        
        hex_str[7+2*i] = ((value[i]>>8)&0xff);
        hex_str[8+2*i] = (value[i] &0xff);
    }
    uint16_t crc = calculateModbusCRC16(hex_str, 7+2*i);
    memcpy(&hex_str[7+2*i], &crc, 2);
    long data_size = 9 + 2*i;
    
    //发送数据前，先清缓存
    OnClearChDataB();
    usleep(50000);
    return OnSetChDataB(hex_str, data_size, ch[right]);

}


uint8_t Jodell_Control::Jodell_modbus_register_read_A(uint16_t start_addr, uint16_t cnt, uint8_t *data)
{
    uint8_t hex_str[256] = {0};

    if (cnt > MAX_OPERATION_REG_NUM)
    {
        printf("cnt invalid\n");
        return 0;
    }

    hex_str[0] = slaveid[left];
    hex_str[1] = func_code_read_reg;
    hex_str[2] = ((start_addr>>8)&0xff);
    hex_str[3] = (start_addr &0xff);
    hex_str[4] = ((cnt>>8)&0xff);
    hex_str[5] = (cnt &0xff);
    uint16_t crc = calculateModbusCRC16(hex_str, 6);

    memcpy(&hex_str[6], &crc, 2);

    usleep(50000);
    //发送数据前，先清缓存
    OnClearChDataA();
    long data_size = 8;
    usleep(50000);
    long target_recv_len = cnt*2 + 5;
    bool result = OnSetChDataA(hex_str, data_size, ch[left]);
    
    if (result)
    {
        int cnt = 100;
        while(cnt--)
        {
            usleep(2000);
            long channel = static_cast<long>(ch[left]);
            long size = OnGetChDataA(data, &channel);
            if (size >= target_recv_len)
            {
                return size;
            }
        }
        return 0;
    }
    else 
    {
        return 0;
    }
    
}


uint8_t Jodell_Control::Jodell_modbus_register_read_B(uint16_t start_addr, uint16_t cnt, uint8_t *data)
{
    uint8_t hex_str[256] = {0};

    if (cnt > MAX_OPERATION_REG_NUM)
    {
        printf("cnt invalid\n");
        return 0;
    }

    hex_str[0] = slaveid[right];
    hex_str[1] = func_code_read_reg;
    hex_str[2] = ((start_addr>>8)&0xff);
    hex_str[3] = (start_addr &0xff);
    hex_str[4] = ((cnt>>8)&0xff);
    hex_str[5] = (cnt &0xff);
    uint16_t crc = calculateModbusCRC16(hex_str, 6);

    memcpy(&hex_str[6], &crc, 2);

    //发送数据前，先清缓存
    OnClearChDataB();
    long data_size = 8;
    usleep(50000);
    long target_recv_len = cnt*2 + 5;
    bool result = OnSetChDataB(hex_str, data_size, ch[right]);

    if (result)
    {
        int cnt = 100;
        while(cnt--)
        {
            usleep(2000);
            long channel = static_cast<long>(ch[right]);
            long size = OnGetChDataB(data, &channel);
            if (size >= target_recv_len)
            {
                return size;
            }
        }
        return 0;
    }
    else 
    {
        return 0;
    }
    
}


/**
 * @brief 计算 Modbus RTU CRC16 校验码
 * 
 * @param data 数据指针
 * @param length 数据长度
 * @return uint16_t 计算出的 CRC16 值
 */
uint16_t Jodell_Control::calculateModbusCRC16(const uint8_t* data, uint32_t length) 
{
    uint16_t crc = 0xFFFF; // 初始值

    for (uint32_t i = 0; i < length; ++i) {
        // 取当前数据字节与 CRC 低字节异或，作为表索引
        uint8_t index = (uint8_t)(crc ^ data[i]);
        // CRC 右移 8 位，并与查表结果异或
        crc = (crc >> 8) ^ crc16_table[index];
    }

    return crc;
}


void Jodell_Control::Jodell_modbus_config_A(uint32_t ch_value, uint8_t id)
{
    ch[left] = ch_value;
    slaveid[left] = id;
}


void Jodell_Control::Jodell_modbus_config_B(uint32_t ch_value, uint8_t id)
{
    ch[right] = ch_value;
    slaveid[right] = id;
}