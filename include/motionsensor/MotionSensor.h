// High level, synchronous C++ API for the HASOMED MotionSensor 2.0.
//
// One request is outstanding at a time (this matches the sensor's
// request/response protocol, see chapter "Protocol basics": "The
// communication is initiated by a PC. The sensor only responds reactively
// to commands."). A background thread continuously reads the serial port,
// reassembles frames and either
//   - completes the currently pending request (Ack or Error), or
//   - forwards unsolicited measurement data packages (RawData14/RawData15)
//     and unsolicited Error packages to user supplied callbacks.
#pragma once

#include "motionsensor/Framing.h"
#include "motionsensor/Protocol.h"
#include "motionsensor/SerialPort.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace motionsensor {

// Thrown when the sensor answers a command with an Error package instead of
// the expected Ack.
class ProtocolError : public std::runtime_error {
public:
    explicit ProtocolError(uint32_t code)
        : std::runtime_error("MotionSensor reported: " + errorCodeToString(code)), code_(code) {}
    uint32_t code() const { return code_; }

private:
    uint32_t code_;
};

// Thrown when no (matching) response was received within the configured
// timeout.
class TimeoutError : public std::runtime_error {
public:
    TimeoutError() : std::runtime_error("MotionSensor: no response within timeout") {}
};

struct StopMeasurementResult {
    uint32_t stopTimeMs = 0;
    uint32_t packageCount = 0;
};

class MotionSensor {
public:
    using ImuDataCallback = std::function<void(const ImuPackage&)>;
    using ErrorCallback = std::function<void(uint32_t errorCode)>;

    MotionSensor() = default;
    ~MotionSensor();

    MotionSensor(const MotionSensor&) = delete;
    MotionSensor& operator=(const MotionSensor&) = delete;

    // Opens the serial connection and starts the background reader thread.
    // Does not yet send Init - call init() explicitly, see "Example of use".
    void connect(const std::string& device, int baudRate = 460800);
    void disconnect();

    // Default timeout used for request/response commands (SetMeasurementMode
    // and SetSensorConfig can take up to ~2s to respond, see manual).
    void setResponseTimeout(int timeoutMs) { responseTimeoutMs_ = timeoutMs; }

    // --- Connection -------------------------------------------------------
    InitInfo init();
    void keepAlive();

    // --- Configuration ------------------------------------------------------
    uint32_t getSerial();
    SensorPosition getSensorPosition();
    CalibrationData getCalibrationData(uint32_t channel, uint32_t mode);
    double getScaleFactor(uint32_t channel, uint32_t mode);
    std::vector<uint32_t> getAvailableMeasurementModes();

    // --- Measurement lifecycle ---------------------------------------------
    // Must be called before StartMeasurement; measurement mode cannot be
    // changed afterwards until UnPrepareMeasurement or StopMeasurement.
    uint32_t prepareMeasurement(); // returns session id
    void unprepareMeasurement();
    uint32_t startMeasurement();   // returns sensor start time [ms]
    StopMeasurementResult stopMeasurement();

    void setMeasurementMode(const MeasurementModeConfig& config);
    MeasurementModeConfig getMeasurementMode();

    bool measurementActive();
    SensorStatsExt sensorStatsExt();

    // --- Streaming measurement data callbacks ------------------------------
    // Invoked from the internal reader thread whenever a RawData14 /
    // RawData15 package arrives. Keep these callbacks fast (e.g. push into a
    // queue) - they run on the I/O thread.
    void onRawData14(ImuDataCallback cb) { rawData14Callback_ = std::move(cb); }
    void onRawData15(ImuDataCallback cb) { rawData15Callback_ = std::move(cb); }

    // Invoked for Error packages that are NOT the response to a pending
    // request (e.g. errors reported spontaneously by the sensor).
    void onUnsolicitedError(ErrorCallback cb) { errorCallback_ = std::move(cb); }

private:
    struct PendingRequest {
        bool active = false;
        uint16_t expectedAck = 0;
        bool completed = false;
        bool isError = false;
        uint32_t errorCode = 0;
        std::vector<uint8_t> data;
    };

    void readLoop();
    void onFrameReceived(const Frame& frame);
    std::vector<uint8_t> request(Command command, const std::vector<uint8_t>& payload,
                                  Command expectedAck, int timeoutMs = -1);

    static ImuPackage parseImuPackage(const std::vector<uint8_t>& data);

    SerialPort serial_;
    FrameDecoder decoder_{[this](const Frame& f) { onFrameReceived(f); }};

    std::thread readerThread_;
    std::atomic<bool> running_{false};

    std::mutex requestMutex_;
    std::condition_variable requestCv_;
    PendingRequest pending_;

    int responseTimeoutMs_ = 3000;

    ImuDataCallback rawData14Callback_;
    ImuDataCallback rawData15Callback_;
    ErrorCallback errorCallback_;
};

} // namespace motionsensor
