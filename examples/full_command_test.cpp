// Exercises the remaining commands not covered by example_of_use.cpp:
// KeepAlive, SensorStatsExt, MeasurementActive, UnPrepareMeasurement and
// RawData15 (single-sample-per-package streaming).
#include "motionsensor/MotionSensor.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

using namespace motionsensor;

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/ttyACM0";
    MotionSensor sensor;

    try {
        sensor.connect(device, 460800);
        InitInfo info = sensor.init();
        std::printf("Init OK (protocol v%u)\n", info.protocolVersion);

        sensor.keepAlive();
        std::printf("KeepAlive OK\n");

        SensorStatsExt stats = sensor.sensorStatsExt();
        std::printf("SensorStatsExt: charge=%u%% ttecp=%umin avgCurrent=%dmA rsoc=%u%% volt=%umV\n",
                    stats.chargeStateOfChargePercent, stats.timeToEmptyMinutes, stats.averageCurrentMa,
                    stats.relativeStateOfChargePercent, stats.voltageMv);

        bool active = sensor.measurementActive();
        std::printf("MeasurementActive (before): %s\n", active ? "true" : "false");

        // PrepareMeasurement + UnPrepareMeasurement without ever starting.
        uint32_t sessionId = sensor.prepareMeasurement();
        std::printf("PrepareMeasurement: session %u\n", sessionId);
        sensor.unprepareMeasurement();
        std::printf("UnPrepareMeasurement OK\n");

        // Now run a short RawData15 measurement (1 sample per package).
        double accScale = sensor.getScaleFactor(static_cast<uint32_t>(Channel::Acc),
                                                 static_cast<uint32_t>(AccMode::g8));
        double gyroScale = sensor.getScaleFactor(static_cast<uint32_t>(Channel::Gyro),
                                                  static_cast<uint32_t>(GyroMode::dps1000));

        std::atomic<uint64_t> count{0};
        sensor.onRawData15([&](const ImuPackage& pkg) {
            count++;
            if (pkg.packageNumber % 100 == 0 && !pkg.samples.empty()) {
                const auto& s = pkg.samples.front();
                std::printf("  RawData15 #%u acc=(%.3f,%.3f,%.3f) m/s^2 gyro=(%.2f,%.2f,%.2f) deg/s\n",
                            pkg.packageNumber, scaleLsbToPhysical(s.accX, accScale),
                            scaleLsbToPhysical(s.accY, accScale), scaleLsbToPhysical(s.accZ, accScale),
                            scaleLsbToPhysical(s.gyroX, gyroScale), scaleLsbToPhysical(s.gyroY, gyroScale),
                            scaleLsbToPhysical(s.gyroZ, gyroScale));
            }
        });

        MeasurementModeConfig cfg;
        cfg.sampleRateHz = 100;
        cfg.measurementMode = static_cast<uint32_t>(Command::RawData15);
        cfg.accMode = static_cast<uint32_t>(AccMode::g8);
        cfg.gyroMode = static_cast<uint32_t>(GyroMode::dps1000);
        cfg.magMode = static_cast<uint32_t>(MagMode::Ga1_3);
        cfg.sendStorageMode = static_cast<uint32_t>(SendStorageMode::SendAndNotStore);
        sensor.setMeasurementMode(cfg);

        sensor.prepareMeasurement();
        sensor.startMeasurement();

        active = sensor.measurementActive();
        std::printf("MeasurementActive (during): %s\n", active ? "true" : "false");

        std::this_thread::sleep_for(std::chrono::seconds(2));

        auto stop = sensor.stopMeasurement();
        std::printf("StopMeasurement: %u packages, %llu RawData15 packages received\n", stop.packageCount,
                    static_cast<unsigned long long>(count.load()));

        active = sensor.measurementActive();
        std::printf("MeasurementActive (after): %s\n", active ? "true" : "false");

        sensor.disconnect();
        std::printf("All commands OK.\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
