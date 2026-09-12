#include "motionsensor/Protocol.h"
#include <sstream>
#include <iomanip>

namespace motionsensor {

std::string errorCodeToString(uint32_t code) {
    switch (static_cast<ErrorCode>(code)) {
        case ErrorCode::BadPackage: return "ErrorBadPackage (package structure broken)";
        case ErrorCode::WrongChecksum: return "ErrorWrongChecksum (package checksum wrong)";
        case ErrorCode::BadData: return "ErrorBadData (wrong data size)";
        case ErrorCode::UnknownPackage: return "ErrorUnknownPackage (command number unknown)";
        case ErrorCode::IwrapNoAnswer: return "ErrorIwrapNoAnswer";
        case ErrorCode::InputBufferFull: return "ErrorInputBufferFull";
        case ErrorCode::InputBufferUnderrun: return "ErrorInputBufferUnderrun";
        case ErrorCode::InputBufferOverrun: return "ErrorInputBufferOverrun";
        case ErrorCode::InputBufferSplit: return "ErrorInputBufferSplit";
        case ErrorCode::MuxRead: return "ErrorMuxRead";
        case ErrorCode::CalibrationWrite: return "ErrorCalibrationWrite";
        case ErrorCode::CalibrationRead: return "ErrorCalibrationRead";
        case ErrorCode::StoreWrite: return "ErrorStoreWrite";
        case ErrorCode::StoreRead: return "ErrorStoreRead";
        case ErrorCode::SdCardUnavailable: return "ErrorSdCardUnavailable (SD-card not inserted or mounted)";
        case ErrorCode::SdCardSessionNotValid: return "ErrorSdCardSessionNotValid";
        case ErrorCode::SdCardCreateSessionFailed: return "ErrorSdCardCreateSessionFailed";
        case ErrorCode::MeasModePrepared: return "ErrorMeasModePrepared (already prepared)";
        case ErrorCode::MeasModeNotPrepared: return "ErrorMeasModeNotPrepared (not prepared)";
        case ErrorCode::TimerRunning: return "ErrorTimerRunning";
        case ErrorCode::TimerNotRunning: return "ErrorTimerNotRunning";
        case ErrorCode::MeasCacheOverrun: return "ErrorMeasCacheOverrun";
        case ErrorCode::Sync: return "ErrorSync (unable to sync sensor with master bt clock)";
        case ErrorCode::InitBma180: return "ErrorInitBma180";
        case ErrorCode::InitImu3000: return "ErrorInitImu3000";
        case ErrorCode::InitHmc5883: return "ErrorInitHmc5883";
        case ErrorCode::InitMpu6050: return "ErrorInitMpu6050";
        case ErrorCode::InitMpu9150: return "ErrorInitMpu9150";
        case ErrorCode::InitLis331hh: return "ErrorInitLis331hh";
        case ErrorCode::InitSdCard: return "ErrorInitSdCard";
    }
    std::ostringstream oss;
    oss << "UnknownError(0x" << std::hex << code << ")";
    return oss.str();
}

std::string sensorPositionToString(uint32_t position) {
    switch (static_cast<SensorPosition>(position)) {
        case SensorPosition::FootLeft: return "FootLeft";
        case SensorPosition::FootRight: return "FootRight";
        case SensorPosition::ShankLeft: return "ShankLeft";
        case SensorPosition::ShankRight: return "ShankRight";
        case SensorPosition::ThighLeft: return "ThighLeft";
        case SensorPosition::ThighRight: return "ThighRight";
        case SensorPosition::Pelvis: return "Pelvis";
        case SensorPosition::Sternum: return "Sternum";
        case SensorPosition::WristLeft: return "WristLeft";
        case SensorPosition::WristRight: return "WristRight";
        case SensorPosition::PelvisDay: return "PelvisDay";
        case SensorPosition::PelvisNight: return "PelvisNight";
    }
    return "Unknown(" + std::to_string(position) + ")";
}

Vec3 applyCalibration(const CalibrationData& calib, const Vec3& scaledLsb) {
    const double dx = scaledLsb.x - calib.bias.x;
    const double dy = scaledLsb.y - calib.bias.y;
    const double dz = scaledLsb.z - calib.bias.z;

    // tmp = (scaledLsb - bias) * RotMat  (row vector times matrix)
    const auto& m = calib.rotationMatrix;
    Vec3 tmp;
    tmp.x = dx * m[0][0] + dy * m[1][0] + dz * m[2][0];
    tmp.y = dx * m[0][1] + dy * m[1][1] + dz * m[2][1];
    tmp.z = dx * m[0][2] + dy * m[1][2] + dz * m[2][2];

    // value_calib = tmp*Cg1 + tmp^2*Cg2 (element-wise per axis)
    Vec3 out;
    out.x = tmp.x * calib.cg1.x + tmp.x * tmp.x * calib.cg2.x;
    out.y = tmp.y * calib.cg1.y + tmp.y * tmp.y * calib.cg2.y;
    out.z = tmp.z * calib.cg1.z + tmp.z * tmp.z * calib.cg2.z;
    return out;
}

} // namespace motionsensor
