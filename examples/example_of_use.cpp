// Example program for the HASOMED MotionSensor 2.0, following the sequence
// described in the manual's "Example of use" chapter:
//
//   Init
//   GetSerial / GetPosition / GetAvailableMeasurementModes           (optional)
//   GetScaleFactor {Acc, Gyro, Mag} / GetCalibrationData {Acc, Gyro, Mag}  (optional)
//   SetMeasurementMode
//   PrepareMeasurement
//   StartMeasurement
//   ... receive measurement packages (here: RawData14) ...
//   StopMeasurement
//
// Usage: motionsensor_example [/dev/ttyACM0] [seconds]

#include "motionsensor/MotionSensor.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

using namespace motionsensor;

namespace {

std::atomic<uint64_t> gSampleCount{0};
std::atomic<uint32_t> gLastPackageNumber{0};

void printCalibration(const char* label, const CalibrationData& c) {
    std::printf("  %-5s bias=(%.4f, %.4f, %.4f)  cg1=(%.4f, %.4f, %.4f)\n", label, c.bias.x, c.bias.y, c.bias.z,
                c.cg1.x, c.cg1.y, c.cg1.z);
}

} // namespace

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/ttyACM0";
    const int measurementSeconds = argc > 2 ? std::atoi(argv[2]) : 5;

    MotionSensor sensor;

    try {
        std::printf("Connecting to %s ...\n", device.c_str());
        sensor.connect(device, 460800);

        // --- Init (mandatory, once per connection) --------------------------
        InitInfo info = sensor.init();
        std::printf("Init: protocol version %u, welcome text: \"%s\"\n", info.protocolVersion,
                    info.welcomeText.c_str());

        // --- Optional: read sensor identity / capabilities ------------------
        uint32_t serial = sensor.getSerial();
        std::printf("Serial number: %u\n", serial);

        SensorPosition position = sensor.getSensorPosition();
        std::printf("Sensor position: %s\n", sensorPositionToString(static_cast<uint32_t>(position)).c_str());

        auto availableModes = sensor.getAvailableMeasurementModes();
        std::printf("Available measurement modes (%zu): ", availableModes.size());
        for (uint32_t m : availableModes) std::printf("0x%02X ", m);
        std::printf("\n");

        const bool rawData14Available =
            std::find(availableModes.begin(), availableModes.end(), static_cast<uint32_t>(Command::RawData14)) !=
            availableModes.end();
        if (!rawData14Available) {
            std::printf("Warning: RawData14 not listed as available on this board revision, trying anyway.\n");
        }

        // Default modes: Acc 8g (channel 0, mode 3), Gyro 1000 deg/s (channel 1,
        // mode 2), Mag 1.3Ga (channel 2, mode 1) - these are the sensor's
        // factory-calibrated channel/mode combinations (see "SetCalibrationData").
        const uint32_t accChannel = static_cast<uint32_t>(Channel::Acc);
        const uint32_t gyroChannel = static_cast<uint32_t>(Channel::Gyro);
        const uint32_t magChannel = static_cast<uint32_t>(Channel::Mag);
        const uint32_t accMode = static_cast<uint32_t>(AccMode::g8);
        const uint32_t gyroMode = static_cast<uint32_t>(GyroMode::dps1000);
        const uint32_t magMode = static_cast<uint32_t>(MagMode::Ga1_3);

        double accScale = sensor.getScaleFactor(accChannel, accMode);
        double gyroScale = sensor.getScaleFactor(gyroChannel, gyroMode);
        double magScale = sensor.getScaleFactor(magChannel, magMode);
        std::printf("Scale factors: acc=%.6f gyro=%.6f mag=%.6f\n", accScale, gyroScale, magScale);

        CalibrationData accCalib = sensor.getCalibrationData(accChannel, accMode);
        CalibrationData gyroCalib = sensor.getCalibrationData(gyroChannel, gyroMode);
        CalibrationData magCalib = sensor.getCalibrationData(magChannel, magMode);
        std::printf("Calibration data:\n");
        printCalibration("Acc", accCalib);
        printCalibration("Gyro", gyroCalib);
        printCalibration("Mag", magCalib);

        // --- Register live data handler for RawData14 -----------------------
        // RawData14 packs 21 samples of Acc+Gyro (no Mag) per package and is
        // recommended for high sample rates (see "Measurement packages").
        sensor.onRawData14([&](const ImuPackage& pkg) {
            gLastPackageNumber = pkg.packageNumber;
            gSampleCount += pkg.samples.size();

            // Print the first sample of every 50th package as a live preview.
            if (pkg.packageNumber % 50 == 0 && !pkg.samples.empty()) {
                const ImuSample& s = pkg.samples.front();
                Vec3 accScaled{scaleLsbToPhysical(s.accX, accScale), scaleLsbToPhysical(s.accY, accScale),
                                scaleLsbToPhysical(s.accZ, accScale)};
                Vec3 gyroScaled{scaleLsbToPhysical(s.gyroX, gyroScale), scaleLsbToPhysical(s.gyroY, gyroScale),
                                 scaleLsbToPhysical(s.gyroZ, gyroScale)};
                Vec3 acc = applyCalibration(accCalib, accScaled);
                Vec3 gyro = applyCalibration(gyroCalib, gyroScaled);
                std::printf("  pkg #%u: acc=(%.3f, %.3f, %.3f) m/s^2  gyro=(%.2f, %.2f, %.2f) deg/s\n",
                            pkg.packageNumber, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
            }
        });

        sensor.onUnsolicitedError([](uint32_t code) {
            std::fprintf(stderr, "Unsolicited sensor error: %s\n", errorCodeToString(code).c_str());
        });

        // --- Configure and run a measurement ---------------------------------
        MeasurementModeConfig modeConfig;
        modeConfig.sampleRateHz = 200;
        modeConfig.measurementMode = static_cast<uint32_t>(Command::RawData14);
        modeConfig.accMode = accMode;
        modeConfig.gyroMode = gyroMode;
        modeConfig.magMode = magMode;
        modeConfig.sendStorageMode = static_cast<uint32_t>(SendStorageMode::SendAndNotStore);

        std::printf("SetMeasurementMode: %u Hz, mode=RawData14\n", modeConfig.sampleRateHz);
        sensor.setMeasurementMode(modeConfig);

        uint32_t sessionId = sensor.prepareMeasurement();
        std::printf("PrepareMeasurement: session id %u\n", sessionId);

        uint32_t startTime = sensor.startMeasurement();
        std::printf("StartMeasurement: start time %u ms. Streaming for %d s ...\n", startTime, measurementSeconds);

        std::this_thread::sleep_for(std::chrono::seconds(measurementSeconds));

        StopMeasurementResult stop = sensor.stopMeasurement();
        std::printf("StopMeasurement: stop time %u ms, %u packages sent by sensor, %llu samples received\n",
                    stop.stopTimeMs, stop.packageCount, static_cast<unsigned long long>(gSampleCount.load()));

        std::printf("Done.\n");
        sensor.disconnect();
        return 0;

    } catch (const ProtocolError& e) {
        std::fprintf(stderr, "Protocol error: %s\n", e.what());
        return 1;
    } catch (const TimeoutError& e) {
        std::fprintf(stderr, "Timeout: %s\n", e.what());
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
