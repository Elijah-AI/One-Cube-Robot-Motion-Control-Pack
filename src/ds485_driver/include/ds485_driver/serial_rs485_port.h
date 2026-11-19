#ifndef DS485_DRIVER_SERIAL_RS485_PORT_H
#define DS485_DRIVER_SERIAL_RS485_PORT_H

/*
 * 第 1 层实现：经 USB-485 串口收发（Linux termios）。
 * 用于台架调试：舵机不装在机械臂上时，用 USB-485 转换器直接接电脑。
 * 固定 8 数据位、无校验、1 停止位，波特率可配置（德晟舵机默认 115200）。
 * 要求转换器自动切换收发方向（常见 CH340/FT232 485 模块都是自动的）。
 */

#include "ds485_driver/rs485_port.h"

#include <string>

class SerialRs485Port : public Rs485Port
{
public:
    SerialRs485Port();
    virtual ~SerialRs485Port();

    /**
     * @brief 打开串口
     * @param device  例如 "/dev/ttyUSB0"
     * @param baud    9600 / 19200 / 38400 / 57600 / 115200 / 230400 / 460800 / 921600
     */
    bool Open(const std::string &device, int baud);
    void Close();
    bool RecoverLink() override;
    bool IsOpen() const { return fd_ >= 0; }

    // Rs485Port 接口
    bool Send(const uint8_t *data, int size) override;
    bool SendBefore(const uint8_t *data, int size, double deadline) override;
    int Receive(uint8_t *buf, int max_size, int timeout_ms) override;
    void Clear() override;
    void SleepMs(int ms) override;
    std::string Name() const override;
    std::string LastError() const override;

private:
    int fd_;
    int baud_=0;
    std::string device_;
    std::string last_error_;
};

#endif
