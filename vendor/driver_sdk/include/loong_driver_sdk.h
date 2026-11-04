/* Copyright 2025 人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#pragma once

#include <cstddef>
#include <vector>
#include <string>

namespace DriverSDK{
struct imuStruct{
    float rpy[3], gyr[3], acc[3];
};

struct sensorStruct{
    float F[3], M[3];
    unsigned int statusCode;
};

struct digitTargetStruct{
    unsigned short pos = 0;             // 0 ~ 90: relaxed ~ tense
};

struct digitActualStruct{
    unsigned short pos = 0;             // 0 ~ 90: relaxed ~ tense
};

struct motorTargetStruct{
    float pos = 0.0, vel = 0.0, tor = 0.0, kp = 0.0, kd = 0.0; // ECAT tor: joint-output Nm
    int enabled = 0;                    // -1: clear error; 0: disable; 1: enable; 2: damp
};

struct motorActualStruct{
    float pos = 0.0, vel = 0.0, tor = 0.0; // ECAT tor: joint-output Nm estimate
    signed char temp[2] = {0, 0};       // Celsius: [0] MOS/drive, [1] rotor.
    // Bus receivers publish normalized Temperature/TemperatureRotor fields.
    // EtherCAT position, velocity, torque and temperature share one feedback node.
    // Unmapped/unsupported channels are 0; wider PDO values saturate to [-128, 127].
    unsigned short statusWord = 65535;  // 0: PREOP; 65535: inactive
    unsigned short errorCode = 0;
    signed char modeDisplay = 0;        // CiA 402 0x6061; valid for EtherCAT drivers
};
static_assert(offsetof(motorActualStruct, statusWord) == 14,
    "motorActualStruct statusWord ABI offset changed");
static_assert(offsetof(motorActualStruct, errorCode) == 16,
    "motorActualStruct errorCode ABI offset changed");
static_assert(offsetof(motorActualStruct, modeDisplay) == 18,
    "motorActualStruct modeDisplay ABI offset changed");
static_assert(sizeof(motorActualStruct) == 20,
    "motorActualStruct ABI size changed");

enum class motorFeedbackSource : unsigned char {
    unavailable = 0,
    torquePermille,   // CiA 402 0x6077, relative to rated torque
    currentPermille,  // CiA 402 0x6078, relative to rated current
    legacyPermille,   // legacy/ambiguous mapping using main-branch scaling
    physicalTorque    // backend already converted feedback to Nm
};

struct motorRawFeedbackStruct {
    int value = 0;
    motorFeedbackSource source = motorFeedbackSource::unavailable;
};

unsigned short single2half(float f);
float half2single(unsigned short u);

class motorSDOClass{
public:
    long value;
    int i;                              // drivers[i]
    short state;                        // -1: error; 0: pending; 1, 2: processing; 3: completed
    unsigned short index;
    unsigned char subindex;
    unsigned char signed_;              // 0: unsigned; 1: signed
    unsigned char bitLength;            // 8, 16 or 32
    unsigned char operation;            // 0: write; 1: read
    motorSDOClass(int i);
    ~motorSDOClass();
};

class motorREGClass{
public:
    long value;
    int i;                              // drivers[i]
    motorREGClass(int i);
    ~motorREGClass();
};

class DriverSDK{
public:
    static DriverSDK& instance();
    void setCPU(unsigned short const cpu);
    int setCPUs(std::vector<unsigned short> const& cpus, std::string const& bus);   // cpus: for EtherCAT0 ~ EtherCAT5 or CAN RX, SocketCAN TX and CANHAL TX; bus: ECAT or CAN
    void setMaxCurr(std::vector<unsigned short> const& maxCurr);
    int setMode(std::vector<char> const& mode);
    void init(char const* xmlFile);
    int getIMUNr();
    int getLeftDigitNr();
    int getRightDigitNr();
    int getTotalMotorNr();
    std::vector<int> getActiveMotors();
    int setCntBias(std::vector<int> const& cntBias);
    int fillSDO(motorSDOClass& data, char const* object);
    void getIMU(imuStruct& data);
    int getIMU(std::vector<imuStruct>& data);
    int getSensor(std::vector<sensorStruct>& data);
    int setDigitTarget(std::vector<digitTargetStruct> const& data);
    int getDigitActual(std::vector<digitActualStruct>& data);
    int setMotorTarget(std::vector<motorTargetStruct> const& data);
    // Reads cyclic feedback only; never queues or consumes temperature SDOs.
    // Check the return value before using data: 0 means all domains complete.
    int getMotorActual(std::vector<motorActualStruct>& data);
    // rawFeedback is captured from the same feedback snapshot as data. Its
    // source identifies whether value is torque permille, current permille,
    // a legacy vendor value, or unavailable/already converted by the backend.
    int getMotorActual(
        std::vector<motorActualStruct>& data,
        std::vector<motorRawFeedbackStruct>& rawFeedback);
    int getEncoderCount(std::vector<int>& data);
    int sendMotorSDORequest(motorSDOClass const& data);
    int recvMotorSDOResponse(motorSDOClass& data);
    int sendMotorREGRequest(motorREGClass const& data);
    int recvMotorREGResponse(motorREGClass& data);
    int calibrate_(int const i);
    int calibrate(int const i);
    void advance();
    // Caller must stop command/acquisition clients after measured disable.
    // Terminal and idempotent: no reinitialization is supported in this process.
    int shutdownEthercat();
    std::string version();
private:
    class impClass;
    impClass& imp;
    int getMotorActualImpl(
        std::vector<motorActualStruct>& data,
        std::vector<motorRawFeedbackStruct>* rawFeedback);
    DriverSDK();
    ~DriverSDK();
    DriverSDK(DriverSDK const&) = delete;
    DriverSDK& operator=(DriverSDK const&) = delete;
};
}