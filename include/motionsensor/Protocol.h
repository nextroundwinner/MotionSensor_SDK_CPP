// Protocol constants and data structures for the HASOMED MotionSensor 2.0
// serial protocol, as described in "MotionSensor_2.0_UserManual_Rev11.pdf".
#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <string>

namespace motionsensor {

// ---------------------------------------------------------------------------
// Framing constants (chapter "Structure of packages")
// ---------------------------------------------------------------------------
namespace framing {
constexpr uint8_t kStartByte = 0xF0;
constexpr uint8_t kStopByte = 0x0F;
constexpr uint8_t kStuffByte = 0x81;
constexpr uint8_t kStuffKey = 0x55;
constexpr size_t kMaxDataLength = 384;
} // namespace framing

// ---------------------------------------------------------------------------
// Command numbers (chapter "Commands")
// ---------------------------------------------------------------------------
enum class Command : uint16_t {
    Init = 0x00,
    InitAck = 0x01,
    KeepAlive = 0x02,
    KeepAliveAck = 0x03,
    ShutDownSensor = 0x04,
    ShutDownSensorAck = 0x05,
    Reset = 0x06,
    ResetAck = 0x07,
    Sleep = 0x08,
    SleepAck = 0x09,
    ChangeLed = 0x14,
    ChangeLedAck = 0x15,
    SwitchOnOff = 0x16,
    SwitchOnOffAck = 0x17,

    Error = 0x2C,
    Debug = 0x30,
    Iwrap = 0x32,
    IwrapAck = 0x33,

    SetSensorConfig = 0x3C,
    SetSensorConfigAck = 0x3D,
    GetSensorConfig = 0x3E,
    GetSensorConfigAck = 0x3F,
    SetSerial = 0x40,
    SetSerialAck = 0x41,
    GetSerial = 0x42,
    GetSerialAck = 0x43,
    SetSensorPosition = 0x44,
    SetSensorPositionAck = 0x45,
    GetSensorPosition = 0x46,
    GetSensorPositionAck = 0x47,
    SetOriginalSensorPosition = 0x48,
    SetOriginalSensorPositionAck = 0x49,
    GetOriginalSensorPosition = 0x4A,
    GetOriginalSensorPositionAck = 0x4B,
    SetCalibrationData = 0x4C,
    SetCalibrationDataAck = 0x4D,
    GetCalibrationData = 0x4E,
    GetCalibrationDataAck = 0x4F,

    GetScaleFactor = 0x54,
    GetScaleFactorAck = 0x55,
    GetAvailableMeasurementModes = 0x56,
    GetAvailableMeasurementModesAck = 0x57,
    PrepareMeasurement = 0x58,
    PrepareMeasurementAck = 0x59,
    UnPrepareMeasurement = 0x5A,
    UnPrepareMeasurementAck = 0x5B,
    StartMeasurement = 0x5C,
    StartMeasurementAck = 0x5D,
    StopMeasurement = 0x5E,
    StopMeasurementAck = 0x5F,
    SetMeasurementMode = 0x60,
    SetMeasurementModeAck = 0x61,
    GetMeasurementMode = 0x62,
    GetMeasurementModeAck = 0x63,
    SensorStats = 0x64,
    SensorStatsAck = 0x65,
    ConfigStandBy = 0x66,
    ConfigStandByAck = 0x67,
    MeasurementActive = 0x68,
    MeasurementActiveAck = 0x69,
    ChangeSendStorageMode = 0x6A,
    ChangeSendStorageModeAck = 0x6B,
    GetCurrentSessionId = 0x6C,
    GetCurrentSessionIdAck = 0x6D,
    GetUsbConnectState = 0x6E,
    GetUsbConnectStateAck = 0x6F,
    GetLink = 0x70,
    GetLinkAck = 0x71,
    SetMeasurementModeExt = 0x72,
    SetMeasurementModeExtAck = 0x73,
    GetMeasurementModeExt = 0x74,
    GetMeasurementModeExtAck = 0x75,
    SensorStatsExt = 0x76,
    SensorStatsExtAck = 0x77,
    GetScaleFactorExt = 0x78,
    GetScaleFactorExtAck = 0x79,

    // Measurement data packages (sensor -> master, no ack)
    RawData3 = 0x7D,
    DebugData = 0x7F,
    CalibData = 0x80,
    OrientationData = 0x81,
    RawData4 = 0x82,
    RawData5 = 0x83,
    RawData6 = 0x84,
    AccComparisonData = 0x85,
    RawData7 = 0x86,
    RawData8 = 0x87,
    SyncData5 = 0x88,
    RtData1 = 0x89, // do not use
    RawData9 = 0x8A,
    RawData10 = 0x8B,
    RawData11 = 0x8C,
    RawData12 = 0x8D,
    GaitPhaseData = 0x8E,
    RawData13 = 0x8F,
    RawData14 = 0x90,
    RawData15 = 0x91,
    RawData16 = 0x92,
    RawData17 = 0x93,

    MeasPackageRequest = 0x96,
    MeasPackageResponse = 0x97,
    MeasPackageUnavailable = 0x98,
    MeasPackageRequestList = 0x9A,
    MeasPackageRequestRange = 0x9C,

    SyncPrepare = 0xA0,
    SyncPrepareAck = 0xA1,
    SyncTime = 0xA2,
    SyncTimeAck = 0xA3,
    SyncOffset = 0xA4,
    SyncOffsetAck = 0xA5,
    SyncBtClock = 0xA6,
    SyncBtClockAck = 0xA7,
    SetRtc = 0xA8,
    SetRtcAck = 0xA9,
    GetRtc = 0xAA,
    GetRtcAck = 0xAB,

    SetBtSettings = 0xB0,
    SetBtSettingsAck = 0xB1,
    GetBtSettings = 0xB2,
    GetBtSettingsAck = 0xB3,
    TestConnection = 0xB4,
    TestConnectionAck = 0xB5,
    SetCalibrationDataExt = 0xB6,
    SetCalibrationDataExtAck = 0xB7,
    GetCalibrationDataExt = 0xB8,
    GetCalibrationDataExtAck = 0xB9,
    ErrorListGet = 0xBA,
    ErrorListGetAck = 0xBB,
    ErrorListReset = 0xBC,
    ErrorListResetAck = 0xBD,

    SdCardGetSectorCount = 0xC0,
    SdCardGetSectorCountAck = 0xC1,
    SdCardGetSessionCount = 0xC2,
    SdCardGetSessionCountAck = 0xC3,
    SdCardGetSessionInfo = 0xC4,
    SdCardGetSessionInfoAck = 0xC5,
    SdCardClearAll = 0xC6,
    SdCardClearAllAck = 0xC7,

    SetMonitorSettings = 0xD0,
};

// ---------------------------------------------------------------------------
// Error constants (chapter "Error constants")
// ---------------------------------------------------------------------------
enum class ErrorCode : uint32_t {
    BadPackage = 0x01,
    WrongChecksum = 0x02,
    BadData = 0x03,
    UnknownPackage = 0x04,
    IwrapNoAnswer = 0xA0,
    InputBufferFull = 0xB0,
    InputBufferUnderrun = 0xB1,
    InputBufferOverrun = 0xB2,
    InputBufferSplit = 0xB3,
    MuxRead = 0xC0,
    CalibrationWrite = 0xD0,
    CalibrationRead = 0xD1,
    StoreWrite = 0xE0,
    StoreRead = 0xE1,
    SdCardUnavailable = 0xE4,
    SdCardSessionNotValid = 0xE5,
    SdCardCreateSessionFailed = 0xE6,
    MeasModePrepared = 0xF0,
    MeasModeNotPrepared = 0xF1,
    TimerRunning = 0xF2,
    TimerNotRunning = 0xF3,
    MeasCacheOverrun = 0xF4,
    Sync = 0xF5,
    InitBma180 = 0x0E00,
    InitImu3000 = 0x0E01,
    InitHmc5883 = 0x0E02,
    InitMpu6050 = 0x0E03,
    InitMpu9150 = 0x0E04,
    InitLis331hh = 0x0E05,
    InitSdCard = 0x0E06,
};

std::string errorCodeToString(uint32_t code);

// ---------------------------------------------------------------------------
// Sensor positions (chapter "Sensor positions")
// ---------------------------------------------------------------------------
enum class SensorPosition : uint32_t {
    FootLeft = 0,
    FootRight = 1,
    ShankLeft = 2,
    ShankRight = 3,
    ThighLeft = 4,
    ThighRight = 5,
    Pelvis = 6,
    Sternum = 7,
    WristLeft = 8,
    WristRight = 9,
    PelvisDay = 10,
    PelvisNight = 11,
};

std::string sensorPositionToString(uint32_t position);

// ---------------------------------------------------------------------------
// Measurement mode of sensors (chapter "Measurement mode of sensors")
// ---------------------------------------------------------------------------
enum class Channel : uint32_t {
    Acc = 0,
    Gyro = 1,
    Mag = 2,
    Pressure = 3,
};

enum class AccMode : uint32_t { g1 = 0, g2 = 1, g4 = 2, g8 = 3, g16 = 4 };
enum class GyroMode : uint32_t { dps250 = 0, dps500 = 1, dps1000 = 2, dps2000 = 3 };
enum class MagMode : uint32_t {
    Ga0_88 = 0,
    Ga1_3 = 1,
    Ga1_9 = 2,
    Ga2_5 = 3,
    Ga4_0 = 4,
    Ga4_7 = 5,
    Ga5_6 = 6,
    Ga8_1 = 7,
};
enum class SendStorageMode : uint32_t {
    SendAndNotStore = 0,
    SendAndStore = 1,
    NotSendAndStore = 2,
};

// ---------------------------------------------------------------------------
// Generic 3D vector / rotation matrix used for calibration data
// ---------------------------------------------------------------------------
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Calibration data as described in "SetCalibrationData"/"GetCalibrationData".
// All values already converted from the on-wire fixed point representation
// (scaled by 1'000'000) to floating point.
struct CalibrationData {
    uint32_t channel = 0;
    uint32_t mode = 0;
    Vec3 bias;
    // rot[row][col], applied as: tmp = (scaledLsb - bias) * RotMat
    std::array<std::array<double, 3>, 3> rotationMatrix{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Vec3 cg1{1.0, 1.0, 1.0};
    Vec3 cg2{0.0, 0.0, 0.0};
};

// Applies the calibration formula described in chapter "Apply calibration
// data" to a value that has already been scaled with the sensor scale
// factor (see scaleLsbToPhysical()).
Vec3 applyCalibration(const CalibrationData& calib, const Vec3& scaledLsb);

// Converts a raw LSB value to a physical value using the scale factor
// returned by GetScaleFactor (chapter "Conversion of LSB values to physical
// values").
inline double scaleLsbToPhysical(int32_t lsb, double scaleFactor) {
    return static_cast<double>(lsb) / scaleFactor;
}

// ---------------------------------------------------------------------------
// Measurement mode configuration (SetMeasurementMode / GetMeasurementMode)
// ---------------------------------------------------------------------------
struct MeasurementModeConfig {
    uint32_t sampleRateHz = 0;
    uint32_t measurementMode = 0; // command number of measurement data package, e.g. Command::RawData14
    uint32_t accMode = 0;
    uint32_t gyroMode = 0;
    uint32_t magMode = 0;
    uint32_t sendStorageMode = 0;
};

// Extended measurement mode configuration (SetMeasurementModeExt / GetMeasurementModeExt)
struct MeasurementModeConfigExt {
    uint32_t sampleRateHz = 0;
    uint32_t measurementMode = 0;
    uint32_t accMode = 0;
    uint32_t gyroMode = 0;
    uint32_t magMode = 0;
    uint32_t pressureMode = 0;
    uint32_t sendStorageMode = 0;
};

struct SensorStatsExt {
    uint32_t chargeStateOfChargePercent = 0; // CSOC
    uint32_t timeToEmptyMinutes = 0;         // TTECP
    int32_t averageCurrentMa = 0;            // AI
    uint32_t relativeStateOfChargePercent = 0; // RSOC
    uint32_t voltageMv = 0;                  // VOLT
};

// ---------------------------------------------------------------------------
// Measurement data packages RawData14 / RawData15
// (Acc 3*16bit + Gyro 3*16bit per sample, 2's complement)
// ---------------------------------------------------------------------------
struct ImuSample {
    int16_t accX = 0, accY = 0, accZ = 0;
    int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
};

struct ImuPackage {
    uint32_t packageNumber = 0;
    std::vector<ImuSample> samples;
};

struct InitInfo {
    uint32_t protocolVersion = 0;
    std::string welcomeText;
};

} // namespace motionsensor
