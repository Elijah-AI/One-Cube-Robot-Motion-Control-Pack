/*
 * 第 1 层实现：USB-485 串口（Linux termios）。
 */
#include "ds485_driver/serial_rs485_port.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>

// 波特率数字 -> termios 常量，不支持返回 0
static speed_t Baud_Const(int baud)
{
    switch (baud)
    {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return 0;
    }
}

SerialRs485Port::SerialRs485Port()
{
    fd_ = -1;
}

SerialRs485Port::~SerialRs485Port()
{
    Close();
}

bool SerialRs485Port::Open(const std::string &device, int baud)
{
    Close();
    device_ = device;
    baud_ = baud;

    speed_t speed = Baud_Const(baud);
    if (speed == 0)
    {
        last_error_ = "不支持的波特率";
        return false;
    }

    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        last_error_ = "打开 " + device + " 失败：" + strerror(errno) + "（检查设备名和 dialout 权限）";
        return false;
    }

    if (ioctl(fd_, TIOCEXCL) < 0)
    {
        last_error_ = "串口独占失败 " + std::string(strerror(errno));
        Close();
        return false;
    }

    // 原始模式 8N1，不做任何字符转换
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0)
    {
        last_error_ = "读取串口参数失败：" + std::string(strerror(errno));
        Close();
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;    // 1 停止位
    tio.c_cflag &= ~PARENB;    // 无校验
    tio.c_cflag &= ~CRTSCTS;   // 无硬件流控
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;        // 8 数据位
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
        last_error_ = "设置串口参数失败：" + std::string(strerror(errno));
        Close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

bool SerialRs485Port::RecoverLink()
{
    // USB 重新插拔后旧 fd 可能一直返回 HUP，按原配置路径重新打开一次。
    if (fd_ >= 0)
    {
        struct pollfd fd{};
        fd.fd = fd_;
        fd.events = POLLIN;
        if (poll(&fd, 1, 0) >= 0 && !(fd.revents & (POLLHUP | POLLERR | POLLNVAL)))
        {
            return true;
        }
    }
    return Open(device_, baud_);
}

void SerialRs485Port::Close()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

bool SerialRs485Port::Send(const uint8_t *data, int size)
{
    return SendBefore(data, size, NowMs() + 300);
}

bool SerialRs485Port::SendBefore(const uint8_t *data, int size, double deadline)
{
    if (fd_ < 0)
    {
        last_error_ = "串口未打开";
        return false;
    }
    int written = 0;
    while (written < size && NowMs() < deadline)
    {
        ssize_t n = write(fd_, data + written, size - written);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                usleep(1000);
                continue;
            }
            last_error_ = "串口写入失败：" + std::string(strerror(errno));
            return false;
        }
        written += (int)n;
    }
    // 自动换向 USB-485：等待内核发送队列排空。适配器内部换向延迟只能从线上测量。
    while (written == size && NowMs() < deadline)
    {
        int pending = 0;
        if (ioctl(fd_, TIOCOUTQ, &pending) < 0)
        {
            break;
        }
        if (pending == 0)
        {
            return true;
        }
        SleepMs(1);
    }
    last_error_ = "串口发送截止时间内未确认排空";
    return false;
}

int SerialRs485Port::Receive(uint8_t *buf, int max_size, int timeout_ms)
{
    if (fd_ < 0)
    {
        last_error_ = "串口未打开";
        return -1;
    }
    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0)
    {
        if (errno == EINTR)
        {
            return 0;
        }
        last_error_ = "串口等待失败：" + std::string(strerror(errno));
        return -1;
    }
    if (ret == 0)
    {
        return 0;   // 超时
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
    {
        last_error_ = "串口已断开或不可用";
        return -1;
    }
    ssize_t n = read(fd_, buf, max_size);
    if (n < 0)
    {
        if (errno == EAGAIN || errno == EINTR)
        {
            return 0;
        }
        last_error_ = "串口读取失败：" + std::string(strerror(errno));
        return -1;
    }
    return (int)n;
}

void SerialRs485Port::Clear()
{
    if (fd_ >= 0)
    {
        tcflush(fd_, TCIFLUSH);
    }
}

void SerialRs485Port::SleepMs(int ms)
{
    if (ms > 0)
    {
        usleep((useconds_t)ms * 1000);
    }
}

std::string SerialRs485Port::Name() const
{
    return "serial " + device_;
}

std::string SerialRs485Port::LastError() const
{
    return last_error_;
}
