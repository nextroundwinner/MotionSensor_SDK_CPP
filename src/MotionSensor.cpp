#include "motionsensor/MotionSensor.h"

#include <cstring>

namespace motionsensor {

namespace {

void appendU32LE(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

uint32_t readU32LE(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<uint32_t>(data.at(offset)) |
           (static_cast<uint32_t>(data.at(offset + 1)) << 8) |
           (static_cast<uint32_t>(data.at(offset + 2)) << 16) |
           (static_cast<uint32_t>(data.at(offset + 3)) << 24);
}

int32_t readI32LE(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int32_t>(readU32LE(data, offset));
}

int16_t readI16LE(const uint8_t* data) {
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

constexpr double kFixedPointScale = 1000000.0; // values scaled by 1'000'000 on the wire

} // namespace

MotionSensor::~MotionSensor() { disconnect(); }

void MotionSensor::connect(const std::string& device, int baudRate) {
    serial_.open(device, baudRate);
    running_ = true;
    readerThread_ = std::thread([this] { readLoop(); });
}

void MotionSensor::disconnect() {
    running_ = false;
    if (readerThread_.joinable()) {
        readerThread_.join();
    }
    serial_.close();

    std::lock_guard<std::mutex> lock(requestMutex_);
    if (pending_.active) {
        pending_.completed = true;
        pending_.isError = false;
        requestCv_.notify_all();
    }
}

void MotionSensor::readLoop() {
    uint8_t buffer[512];
    while (running_) {
        size_t n;
        try {
            n = serial_.read(buffer, sizeof(buffer), 200 /*ms*/);
        } catch (const std::exception&) {
            break; // device gone
        }
        if (n > 0) {
            decoder_.feed(buffer, n);
        }
    }
}

void MotionSensor::onFrameReceived(const Frame& frame) {
    if (frame.command == static_cast<uint16_t>(Command::Error)) {
        uint32_t errorCode = frame.data.size() >= 4 ? readU32LE(frame.data, 0) : 0;

        std::unique_lock<std::mutex> lock(requestMutex_);
        if (pending_.active && !pending_.completed) {
            pending_.isError = true;
            pending_.errorCode = errorCode;
            pending_.completed = true;
            lock.unlock();
            requestCv_.notify_all();
        } else {
            lock.unlock();
            if (errorCallback_) errorCallback_(errorCode);
        }
        return;
    }

    if (frame.command == static_cast<uint16_t>(Command::RawData14)) {
        if (rawData14Callback_) rawData14Callback_(parseImuPackage(frame.data));
        return;
    }
    if (frame.command == static_cast<uint16_t>(Command::RawData15)) {
        if (rawData15Callback_) rawData15Callback_(parseImuPackage(frame.data));
        return;
    }

    std::unique_lock<std::mutex> lock(requestMutex_);
    if (pending_.active && !pending_.completed && frame.command == pending_.expectedAck) {
        pending_.data = frame.data;
        pending_.completed = true;
        lock.unlock();
        requestCv_.notify_all();
    }
    // else: unsolicited / unexpected frame - ignored.
}

std::vector<uint8_t> MotionSensor::request(Command command, const std::vector<uint8_t>& payload,
                                            Command expectedAck, int timeoutMs) {
    if (timeoutMs < 0) timeoutMs = responseTimeoutMs_;

    std::unique_lock<std::mutex> lock(requestMutex_);
    if (pending_.active) {
        throw std::logic_error("MotionSensor: a request is already in flight (single outstanding request protocol)");
    }
    pending_ = PendingRequest{};
    pending_.active = true;
    pending_.expectedAck = static_cast<uint16_t>(expectedAck);

    auto frame = encodeFrame(static_cast<uint16_t>(command), payload);
    serial_.write(frame);

    bool ok = requestCv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                   [this] { return pending_.completed; });
    pending_.active = false;
    if (!ok) {
        throw TimeoutError();
    }
    if (pending_.isError) {
        uint32_t code = pending_.errorCode;
        lock.unlock();
        throw ProtocolError(code);
    }
    return pending_.data;
}

InitInfo MotionSensor::init() {
    auto data = request(Command::Init, {}, Command::InitAck);
    InitInfo info;
    info.protocolVersion = readU32LE(data, 0);
    uint32_t textLen = readU32LE(data, 4);
    if (data.size() >= 8 + textLen) {
        info.welcomeText.assign(reinterpret_cast<const char*>(data.data() + 8), textLen);
    }
    return info;
}

void MotionSensor::keepAlive() { request(Command::KeepAlive, {}, Command::KeepAliveAck); }

uint32_t MotionSensor::getSerial() {
    auto data = request(Command::GetSerial, {}, Command::GetSerialAck);
    return readU32LE(data, 0);
}

SensorPosition MotionSensor::getSensorPosition() {
    auto data = request(Command::GetSensorPosition, {}, Command::GetSensorPositionAck);
    return static_cast<SensorPosition>(readU32LE(data, 0));
}

CalibrationData MotionSensor::getCalibrationData(uint32_t channel, uint32_t mode) {
    std::vector<uint8_t> payload;
    appendU32LE(payload, channel);
    appendU32LE(payload, mode);
    auto data = request(Command::GetCalibrationData, payload, Command::GetCalibrationDataAck);

    CalibrationData calib;
    calib.channel = readU32LE(data, 0);
    calib.mode = readU32LE(data, 4);
    calib.bias.x = readI32LE(data, 8) / kFixedPointScale;
    calib.bias.y = readI32LE(data, 12) / kFixedPointScale;
    calib.bias.z = readI32LE(data, 16) / kFixedPointScale;
    calib.rotationMatrix[0][0] = readI32LE(data, 20) / kFixedPointScale; // XX
    calib.rotationMatrix[0][1] = readI32LE(data, 24) / kFixedPointScale; // XY
    calib.rotationMatrix[0][2] = readI32LE(data, 28) / kFixedPointScale; // XZ
    calib.rotationMatrix[1][0] = readI32LE(data, 32) / kFixedPointScale; // YX
    calib.rotationMatrix[1][1] = readI32LE(data, 36) / kFixedPointScale; // YY
    calib.rotationMatrix[1][2] = readI32LE(data, 40) / kFixedPointScale; // YZ
    calib.rotationMatrix[2][0] = readI32LE(data, 44) / kFixedPointScale; // ZX
    calib.rotationMatrix[2][1] = readI32LE(data, 48) / kFixedPointScale; // ZY
    calib.rotationMatrix[2][2] = readI32LE(data, 52) / kFixedPointScale; // ZZ
    calib.cg1.x = readI32LE(data, 56) / kFixedPointScale;
    calib.cg1.y = readI32LE(data, 60) / kFixedPointScale;
    calib.cg1.z = readI32LE(data, 64) / kFixedPointScale;
    calib.cg2.x = readI32LE(data, 68) / kFixedPointScale;
    calib.cg2.y = readI32LE(data, 72) / kFixedPointScale;
    calib.cg2.z = readI32LE(data, 76) / kFixedPointScale;
    return calib;
}

double MotionSensor::getScaleFactor(uint32_t channel, uint32_t mode) {
    std::vector<uint8_t> payload;
    appendU32LE(payload, channel);
    appendU32LE(payload, mode);
    auto data = request(Command::GetScaleFactor, payload, Command::GetScaleFactorAck);
    int32_t scaled = readI32LE(data, 8);
    return scaled / kFixedPointScale;
}

std::vector<uint32_t> MotionSensor::getAvailableMeasurementModes() {
    auto data = request(Command::GetAvailableMeasurementModes, {}, Command::GetAvailableMeasurementModesAck);
    uint32_t count = readU32LE(data, 0);
    std::vector<uint32_t> modes;
    modes.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        modes.push_back(readU32LE(data, 4 + i * 4));
    }
    return modes;
}

uint32_t MotionSensor::prepareMeasurement() {
    auto data = request(Command::PrepareMeasurement, {}, Command::PrepareMeasurementAck);
    return readU32LE(data, 0);
}

void MotionSensor::unprepareMeasurement() {
    request(Command::UnPrepareMeasurement, {}, Command::UnPrepareMeasurementAck);
}

uint32_t MotionSensor::startMeasurement() {
    auto data = request(Command::StartMeasurement, {}, Command::StartMeasurementAck);
    return readU32LE(data, 0);
}

StopMeasurementResult MotionSensor::stopMeasurement() {
    auto data = request(Command::StopMeasurement, {}, Command::StopMeasurementAck);
    StopMeasurementResult result;
    result.stopTimeMs = readU32LE(data, 0);
    result.packageCount = readU32LE(data, 4);
    return result;
}

void MotionSensor::setMeasurementMode(const MeasurementModeConfig& config) {
    std::vector<uint8_t> payload;
    appendU32LE(payload, config.sampleRateHz);
    appendU32LE(payload, config.measurementMode);
    appendU32LE(payload, config.accMode);
    appendU32LE(payload, config.gyroMode);
    appendU32LE(payload, config.magMode);
    appendU32LE(payload, config.sendStorageMode);
    // SetMeasurementMode may reinitialize internal sensors, allow extra time.
    request(Command::SetMeasurementMode, payload, Command::SetMeasurementModeAck, std::max(responseTimeoutMs_, 2000));
}

MeasurementModeConfig MotionSensor::getMeasurementMode() {
    auto data = request(Command::GetMeasurementMode, {}, Command::GetMeasurementModeAck);
    MeasurementModeConfig config;
    config.sampleRateHz = readU32LE(data, 0);
    config.measurementMode = readU32LE(data, 4);
    config.accMode = readU32LE(data, 8);
    config.gyroMode = readU32LE(data, 12);
    config.magMode = readU32LE(data, 16);
    config.sendStorageMode = readU32LE(data, 20);
    return config;
}

bool MotionSensor::measurementActive() {
    auto data = request(Command::MeasurementActive, {}, Command::MeasurementActiveAck);
    return readU32LE(data, 0) != 0;
}

SensorStatsExt MotionSensor::sensorStatsExt() {
    auto data = request(Command::SensorStatsExt, {}, Command::SensorStatsExtAck);
    SensorStatsExt stats;
    stats.chargeStateOfChargePercent = readU32LE(data, 0);
    stats.timeToEmptyMinutes = readU32LE(data, 4);
    stats.averageCurrentMa = readI32LE(data, 8);
    stats.relativeStateOfChargePercent = readU32LE(data, 12);
    stats.voltageMv = readU32LE(data, 16);
    return stats;
}

ImuPackage MotionSensor::parseImuPackage(const std::vector<uint8_t>& data) {
    ImuPackage pkg;
    if (data.size() < 4) return pkg;
    pkg.packageNumber = readU32LE(data, 0);

    constexpr size_t kSampleSize = 12; // 6 * int16
    size_t offset = 4;
    while (offset + kSampleSize <= data.size()) {
        const uint8_t* p = data.data() + offset;
        ImuSample sample;
        sample.accX = readI16LE(p + 0);
        sample.accY = readI16LE(p + 2);
        sample.accZ = readI16LE(p + 4);
        sample.gyroX = readI16LE(p + 6);
        sample.gyroY = readI16LE(p + 8);
        sample.gyroZ = readI16LE(p + 10);
        pkg.samples.push_back(sample);
        offset += kSampleSize;
    }
    return pkg;
}

} // namespace motionsensor
