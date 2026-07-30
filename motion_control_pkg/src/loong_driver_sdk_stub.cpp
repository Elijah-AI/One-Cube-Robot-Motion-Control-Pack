#include "loong_driver_sdk.h"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace DriverSDK {

namespace {

constexpr int kDefaultMotorCount = 32;

std::vector<int> buildActiveMotors(int motor_count)
{
    std::vector<int> active_motors;
    active_motors.reserve(static_cast<std::size_t>(std::max(motor_count, 0)));
    for (int i = 0; i < motor_count; ++i) {
        active_motors.push_back(i);
    }
    return active_motors;
}

}  // namespace

class DriverSDK::impClass {
public:
    std::mutex mutex;
    unsigned short cpu = 0;
    std::vector<unsigned short> cpus;
    std::string bus = "ECAT";
    std::string config_path;
    std::vector<unsigned short> max_curr;
    std::vector<char> mode;
    std::vector<int> cnt_bias;
    std::vector<motorTargetStruct> target;
    std::vector<motorActualStruct> actual;
    std::vector<int> encoder_count;
    std::vector<imuStruct> imu_data = std::vector<imuStruct>(1);
    std::vector<sensorStruct> sensor_data;
    std::vector<digitTargetStruct> left_digit_target;
    std::vector<digitTargetStruct> right_digit_target;
    std::vector<digitActualStruct> left_digit_actual;
    std::vector<digitActualStruct> right_digit_actual;
    int motor_count = kDefaultMotorCount;

    void ensureMotorStorageLocked(std::size_t requested_count)
    {
        const std::size_t normalized_count = std::max<std::size_t>(
            requested_count,
            static_cast<std::size_t>(std::max(motor_count, 1)));

        motor_count = static_cast<int>(normalized_count);
        target.resize(normalized_count);
        actual.resize(normalized_count);
        encoder_count.resize(normalized_count, 0);
        if (mode.size() < normalized_count) {
            mode.resize(normalized_count, 0);
        }
        if (cnt_bias.size() < normalized_count) {
            cnt_bias.resize(normalized_count, 0);
        }
        if (max_curr.size() < normalized_count) {
            max_curr.resize(normalized_count, 0);
        }
    }
};

unsigned short single2half(float f)
{
    // Stub conversion: keep 16 high bits so tests can round-trip deterministically.
    unsigned int raw = 0;
    std::memcpy(&raw, &f, sizeof(raw));
    return static_cast<unsigned short>(raw >> 16U);
}

float half2single(unsigned short u)
{
    unsigned int raw = static_cast<unsigned int>(u) << 16U;
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

motorSDOClass::motorSDOClass(int i_)
    : value(0),
      i(i_),
      state(0),
      index(0),
      subindex(0),
      signed_(0),
      bitLength(32),
      operation(0)
{
}

motorSDOClass::~motorSDOClass() = default;

motorREGClass::motorREGClass(int i_) : value(0), i(i_) {}

motorREGClass::~motorREGClass() = default;

DriverSDK &DriverSDK::instance()
{
    static DriverSDK sdk;
    return sdk;
}

DriverSDK::DriverSDK() : imp(*new impClass()) {}

DriverSDK::~DriverSDK()
{
    delete &imp;
}

void DriverSDK::setCPU(unsigned short const cpu)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.cpu = cpu;
}

int DriverSDK::setCPUs(std::vector<unsigned short> const &cpus, std::string const &bus)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.cpus = cpus;
    imp.bus = bus;
    return 0;
}

void DriverSDK::setMaxCurr(std::vector<unsigned short> const &maxCurr)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.max_curr = maxCurr;
    imp.ensureMotorStorageLocked(maxCurr.size());
}

int DriverSDK::setMode(std::vector<char> const &mode)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.mode = mode;
    if (!mode.empty()) {
        imp.motor_count = static_cast<int>(mode.size());
    }
    imp.ensureMotorStorageLocked(mode.size());
    return 0;
}

void DriverSDK::init(char const *xmlFile)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.config_path = xmlFile != nullptr ? xmlFile : "";
    imp.ensureMotorStorageLocked(static_cast<std::size_t>(imp.motor_count));
}

int DriverSDK::getIMUNr()
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    return static_cast<int>(imp.imu_data.size());
}

int DriverSDK::getLeftDigitNr()
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    return static_cast<int>(imp.left_digit_actual.size());
}

int DriverSDK::getRightDigitNr()
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    return static_cast<int>(imp.right_digit_actual.size());
}

int DriverSDK::getTotalMotorNr()
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    return imp.motor_count;
}

std::vector<int> DriverSDK::getActiveMotors()
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    return buildActiveMotors(imp.motor_count);
}

int DriverSDK::setCntBias(std::vector<int> const &cntBias)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.cnt_bias = cntBias;
    imp.ensureMotorStorageLocked(cntBias.size());
    return 0;
}

int DriverSDK::fillSDO(motorSDOClass &data, char const *object)
{
    (void)object;
    data.state = 0;
    return 0;
}

void DriverSDK::getIMU(imuStruct &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    if (imp.imu_data.empty()) {
        data = imuStruct();
        return;
    }
    data = imp.imu_data.front();
}

int DriverSDK::getIMU(std::vector<imuStruct> &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    data = imp.imu_data;
    return 0;
}

int DriverSDK::getSensor(std::vector<sensorStruct> &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    data = imp.sensor_data;
    return 0;
}

int DriverSDK::setDigitTarget(std::vector<digitTargetStruct> const &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.left_digit_target = data;
    imp.left_digit_actual.resize(data.size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        imp.left_digit_actual[i].pos = data[i].pos;
    }
    return 0;
}

int DriverSDK::getDigitActual(std::vector<digitActualStruct> &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    data = imp.left_digit_actual;
    return 0;
}

int DriverSDK::setMotorTarget(std::vector<motorTargetStruct> const &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    if (!data.empty()) {
        imp.motor_count = static_cast<int>(data.size());
        imp.ensureMotorStorageLocked(data.size());
        imp.target = data;
    }
    return 0;
}

int DriverSDK::getMotorActual(std::vector<motorActualStruct> &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    const std::size_t requested_size = data.empty() ? static_cast<std::size_t>(imp.motor_count) : data.size();
    imp.ensureMotorStorageLocked(requested_size);
    data.assign(imp.actual.begin(), imp.actual.begin() + static_cast<long>(requested_size));
    return 0;
}

int DriverSDK::getEncoderCount(std::vector<int> &data)
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.ensureMotorStorageLocked(static_cast<std::size_t>(imp.motor_count));
    data = imp.encoder_count;
    return 0;
}

int DriverSDK::sendMotorSDORequest(motorSDOClass const &data)
{
    (void)data;
    return 0;
}

int DriverSDK::recvMotorSDOResponse(motorSDOClass &data)
{
    data.state = 3;
    return 0;
}

int DriverSDK::sendMotorREGRequest(motorREGClass const &data)
{
    (void)data;
    return 0;
}

int DriverSDK::recvMotorREGResponse(motorREGClass &data)
{
    data.value = 0;
    return 0;
}

int DriverSDK::calibrate_(int const i)
{
    (void)i;
    return 0;
}

int DriverSDK::calibrate(int const i)
{
    (void)i;
    return 0;
}

void DriverSDK::advance()
{
    std::lock_guard<std::mutex> lock(imp.mutex);
    imp.ensureMotorStorageLocked(static_cast<std::size_t>(imp.motor_count));

    for (std::size_t i = 0; i < imp.actual.size() && i < imp.target.size(); ++i) {
        if (imp.target[i].enabled != 0) {
            imp.actual[i].pos = imp.target[i].pos;
            imp.actual[i].vel = imp.target[i].vel;
            imp.actual[i].tor = imp.target[i].tor;
            imp.actual[i].statusWord = 4;
        } else {
            imp.actual[i].vel = 0.0f;
            imp.actual[i].tor = 0.0f;
            imp.actual[i].statusWord = 0;
        }
        imp.actual[i].errorCode = 0;
        imp.actual[i].temp[0] = 0;
        imp.actual[i].temp[1] = 0;
        imp.encoder_count[i] = imp.cnt_bias[i];
    }
}

std::string DriverSDK::version()
{
    return "loong_driver_sdk_stub";
}

}  // namespace DriverSDK
