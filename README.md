# MotionSensor SDK (C++)

A small, dependency-free C++17 library for talking to the **HASOMED MotionSensor 2.0**
over its serial protocol (USB-CDC at `/dev/ttyACM0`, or a Bluetooth SPP virtual
COM port). It implements the framing (start/stop byte, byte-stuffing,
checksum) and a synchronous request/response API for the commands needed to
configure the sensor and stream IMU data, as specified in
`MotionSensor_2.0_UserManual_Rev11.pdf`.

Framing and the request/response layer are verified against the manual's
worked stuffing example and against a physical sensor (see
[Examples](#examples)).

## Contents

- [Requirements](#requirements)
- [Build](#build)
- [Project layout](#project-layout)
- [Quick start](#quick-start)
- [API reference](#api-reference)
  - [Connecting](#connecting)
  - [Init](#init)
  - [KeepAlive](#keepalive)
  - [GetSerial](#getserial)
  - [GetSensorPosition](#getsensorposition)
  - [GetCalibrationData](#getcalibrationdata)
  - [GetScaleFactor](#getscalefactor)
  - [GetAvailableMeasurementModes](#getavailablemeasurementmodes)
  - [SetMeasurementMode / GetMeasurementMode](#setmeasurementmode--getmeasurementmode)
  - [PrepareMeasurement / UnPrepareMeasurement](#preparemeasurement--unpreparemeasurement)
  - [StartMeasurement / StopMeasurement](#startmeasurement--stopmeasurement)
  - [MeasurementActive](#measurementactive)
  - [SensorStatsExt](#sensorstatsext)
  - [RawData14 / RawData15 (streaming)](#rawdata14--rawdata15-streaming)
  - [Error handling](#error-handling)
- [Converting raw samples to physical units](#converting-raw-samples-to-physical-units)
- [Examples](#examples)
- [Known device quirks](#known-device-quirks)

## Requirements

- Linux (uses POSIX `termios`/`poll`)
- g++ with C++17 support
- Read/write access to the serial device, e.g. add your user to the
  `dialout` group: `sudo usermod -aG dialout $USER` (re-login required)

## Build

```sh
make            # builds build/libmotionsensor.a and build/example_of_use
./build/example_of_use /dev/ttyACM0 5   # device path, measurement duration in seconds
```

Other build targets produced by `make`:

| Binary                        | Source                              | Purpose                                                   |
|--------------------------------|--------------------------------------|------------------------------------------------------------|
| `build/example_of_use`         | `examples/example_of_use.cpp`        | Full walkthrough following the manual's "Example of use"   |

To also build the extra test programs described under [Examples](#examples):

```sh
g++ -std=c++17 -O2 -Iinclude examples/full_command_test.cpp -o build/full_command_test -Lbuild -lmotionsensor -lpthread
g++ -std=c++17 -O2 -Iinclude examples/framing_selftest.cpp   -o build/framing_selftest   -Lbuild -lmotionsensor -lpthread
```

## Project layout

```
include/motionsensor/
  Protocol.h      Command numbers, error codes, enums, data structures
  Framing.h       Byte-stuffing, checksum, frame encode/decode
  SerialPort.h    Raw-mode termios wrapper
  MotionSensor.h  High level API (this is what you #include)
src/              Implementation of the above
examples/         Example programs (see below)
```

## Quick start

```cpp
#include "motionsensor/MotionSensor.h"
using namespace motionsensor;

MotionSensor sensor;
sensor.connect("/dev/ttyACM0", 460800);

InitInfo info = sensor.init();
uint32_t serial = sensor.getSerial();

MeasurementModeConfig cfg;
cfg.sampleRateHz    = 200;
cfg.measurementMode = static_cast<uint32_t>(Command::RawData14);
cfg.accMode         = static_cast<uint32_t>(AccMode::g8);
cfg.gyroMode        = static_cast<uint32_t>(GyroMode::dps1000);
cfg.magMode         = static_cast<uint32_t>(MagMode::Ga1_3);
cfg.sendStorageMode = static_cast<uint32_t>(SendStorageMode::SendAndNotStore);
sensor.setMeasurementMode(cfg);

sensor.onRawData14([](const ImuPackage& pkg) {
    for (const auto& s : pkg.samples) {
        // s.accX/Y/Z, s.gyroX/Y/Z are raw LSB values, see "Converting raw
        // samples to physical units" below.
    }
});

sensor.prepareMeasurement();
sensor.startMeasurement();
// ... let it stream ...
sensor.stopMeasurement();
sensor.disconnect();
```

All calls throw on failure: `motionsensor::ProtocolError` when the sensor
answers with an `Error` package, `motionsensor::TimeoutError` when no
response arrives in time, and `std::runtime_error` for serial I/O failures.

## API reference

All commands live on `motionsensor::MotionSensor` (`include/motionsensor/MotionSensor.h`).
Every method sends the named command and blocks until the sensor's `...Ack`
package arrives, translating it into a return value; the on-wire command
numbers and payload layouts are defined in `include/motionsensor/Protocol.h`
and match the manual's "Commands" chapter.

### Connecting

```cpp
void connect(const std::string& device, int baudRate = 460800);
void disconnect();
void setResponseTimeout(int timeoutMs); // default 3000
```

`connect()` opens the serial port (8N1, no flow control, see manual chapter
"Serial connection") and starts a background thread that continuously reads
and decodes frames. `disconnect()` stops the thread and closes the port. Call
`setResponseTimeout()` before issuing commands if you need a longer/shorter
Ack timeout than the 3 s default (`SetMeasurementMode` already gets a
built-in minimum of 2 s since the sensor may reinitialize internal chips).

### Init

```cpp
InitInfo init();
```

Sends `Init` (0x00). Must be called once after `connect()`, before any other
command. Returns:

```cpp
struct InitInfo {
    uint32_t protocolVersion;
    std::string welcomeText;
};
```

### KeepAlive

```cpp
void keepAlive();
```

Sends `KeepAlive` (0x02). Resets the sensor's standby timer (see
`ConfigStandBy`/`SetMonitorSettings` in the manual); call periodically if you
are not otherwise communicating with the sensor to prevent it from powering
down.

### GetSerial

```cpp
uint32_t getSerial();
```

Sends `GetSerial` (0x42). Returns the sensor's serial number, printed on its
case label.

### GetSensorPosition

```cpp
SensorPosition getSensorPosition();
```

Sends `GetSensorPosition` (0x46). Returns the body position the sensor is
configured for:

```cpp
enum class SensorPosition : uint32_t {
    FootLeft = 0, FootRight = 1, ShankLeft = 2, ShankRight = 3,
    ThighLeft = 4, ThighRight = 5, Pelvis = 6, Sternum = 7,
    WristLeft = 8, WristRight = 9, PelvisDay = 10, PelvisNight = 11,
};
```

Use `sensorPositionToString(uint32_t)` for a human-readable name.

### GetCalibrationData

```cpp
CalibrationData getCalibrationData(uint32_t channel, uint32_t mode);
```

Sends `GetCalibrationData` (0x4E) for the given sensor chip channel and
measurement range mode. Only the following `channel`/`mode` combinations are
factory-calibrated (see manual "SetCalibrationData"):

| channel          | mode                              |
|------------------|------------------------------------|
| `Channel::Acc`   | `AccMode::g8` or `AccMode::g16`    |
| `Channel::Gyro`  | `GyroMode::dps1000` or `dps2000`   |
| `Channel::Mag`   | `MagMode::Ga1_3`                   |

Returns already-descaled floating point values (the wire format is a fixed
point integer scaled by 1,000,000):

```cpp
struct CalibrationData {
    uint32_t channel, mode;
    Vec3 bias;                                  // per-axis bias
    std::array<std::array<double,3>,3> rotationMatrix; // rot[row][col]
    Vec3 cg1, cg2;                               // 1st/2nd order gain correction
};
```

Throws `ProtocolError` with code `ErrorCalibrationRead` if no calibration is
stored for the requested channel/mode.

### GetScaleFactor

```cpp
double getScaleFactor(uint32_t channel, uint32_t mode);
```

Sends `GetScaleFactor` (0x54). Returns the scale factor (already divided by
1,000,000) used to convert a raw LSB value into a physical unit, see
[Converting raw samples to physical units](#converting-raw-samples-to-physical-units).

### GetAvailableMeasurementModes

```cpp
std::vector<uint32_t> getAvailableMeasurementModes();
```

Sends `GetAvailableMeasurementModes` (0x56). Returns the list of measurement
data command numbers (`Command::RawData14`, `Command::RawData15`, ...) this
particular sensor's firmware/board revision supports; not every board
revision supports every `RawDataXX` mode (see manual "Measurement packages" /
"Availability").

### SetMeasurementMode / GetMeasurementMode

```cpp
void setMeasurementMode(const MeasurementModeConfig& config);
MeasurementModeConfig getMeasurementMode();
```

Sends `SetMeasurementMode` (0x60) / `GetMeasurementMode` (0x62). The sensor
must be **unprepared** when calling `setMeasurementMode()` (i.e. before
`prepareMeasurement()`, or after `unprepareMeasurement()`/`stopMeasurement()`).

```cpp
struct MeasurementModeConfig {
    uint32_t sampleRateHz;     // up to 600 Hz
    uint32_t measurementMode;  // command number, e.g. Command::RawData14
    uint32_t accMode;          // AccMode
    uint32_t gyroMode;         // GyroMode
    uint32_t magMode;          // MagMode
    uint32_t sendStorageMode;  // SendStorageMode
};
```

### PrepareMeasurement / UnPrepareMeasurement

```cpp
uint32_t prepareMeasurement();   // returns session id
void unprepareMeasurement();
```

Sends `PrepareMeasurement` (0x58) / `UnPrepareMeasurement` (0x5A).
`prepareMeasurement()` must be called before every `startMeasurement()`; it
resets the package counter and starts an internal-storage session, returning
a session id that later can be used to request retransmission of lost
packages (`MeasPackageRequest`, not exposed by this library). Calling
`stopMeasurement()` implicitly un-prepares the sensor, so
`unprepareMeasurement()` is only needed if you prepared but never started a
measurement.

### StartMeasurement / StopMeasurement

```cpp
uint32_t startMeasurement();               // returns sensor start time [ms]
StopMeasurementResult stopMeasurement();
```

Sends `StartMeasurement` (0x5C) / `StopMeasurement` (0x5E). Once started, the
sensor streams measurement data packages (see below) until
`stopMeasurement()` is called.

```cpp
struct StopMeasurementResult {
    uint32_t stopTimeMs;
    uint32_t packageCount; // number of measurement data packages sent
};
```

### MeasurementActive

```cpp
bool measurementActive();
```

Sends `MeasurementActive` (0x68). Returns whether a measurement is currently
running.

### SensorStatsExt

```cpp
SensorStatsExt sensorStatsExt();
```

Sends `SensorStatsExt` (0x76). Returns battery/charging status obtained from
the Bluetooth module:

```cpp
struct SensorStatsExt {
    uint32_t chargeStateOfChargePercent;   // CSOC
    uint32_t timeToEmptyMinutes;           // TTECP
    int32_t  averageCurrentMa;             // AI
    uint32_t relativeStateOfChargePercent; // RSOC
    uint32_t voltageMv;                    // VOLT
};
```

### RawData14 / RawData15 (streaming)

```cpp
void onRawData14(std::function<void(const ImuPackage&)> cb);
void onRawData15(std::function<void(const ImuPackage&)> cb);
```

Registers a callback for the corresponding measurement data package,
invoked **on the internal reader thread** whenever one arrives while a
measurement is active (select the package type via `measurementMode` in
`SetMeasurementMode`). Keep the callback fast (e.g. push samples into a
lock-free queue) since it blocks further frame processing while it runs.

```cpp
struct ImuSample {
    int16_t accX, accY, accZ;   // raw LSB, 2's complement
    int16_t gyroX, gyroY, gyroZ;
};
struct ImuPackage {
    uint32_t packageNumber;     // unique, increasing id per package
    std::vector<ImuSample> samples; // RawData14: 21 samples, RawData15: 1 sample
};
```

Both packages contain accelerometer + gyroscope samples only (no
magnetometer); `RawData14` batches 21 samples per package for efficient
high-rate streaming, `RawData15` sends one sample per package for the lowest
possible latency.

### Error handling

```cpp
void onUnsolicitedError(std::function<void(uint32_t errorCode)> cb);
```

Every request-issuing method above throws `motionsensor::ProtocolError` (with
`.code()` and a human-readable `.what()` via `errorCodeToString()`) if the
sensor answers with an `Error` package instead of the expected `...Ack`, and
`motionsensor::TimeoutError` if no response arrives within the configured
timeout. `onUnsolicitedError()` additionally catches `Error` packages the
sensor sends spontaneously (i.e. not as the direct answer to a pending
command).

## Converting raw samples to physical units

As described in the manual ("Conversion of measurement data (packages:
RawDataXX)"):

```cpp
double scaleLsbToPhysical(int32_t lsb, double scaleFactor); // = lsb / scaleFactor
Vec3   applyCalibration(const CalibrationData& calib, const Vec3& scaledLsb);
```

```cpp
uint32_t accChannel = static_cast<uint32_t>(Channel::Acc);
uint32_t accMode    = static_cast<uint32_t>(AccMode::g8);
uint32_t gyroChannel = static_cast<uint32_t>(Channel::Gyro);
uint32_t gyroMode    = static_cast<uint32_t>(GyroMode::dps1000);

double accScale  = sensor.getScaleFactor(accChannel, accMode);
double gyroScale = sensor.getScaleFactor(gyroChannel, gyroMode);
CalibrationData accCalib  = sensor.getCalibrationData(accChannel, accMode);
CalibrationData gyroCalib = sensor.getCalibrationData(gyroChannel, gyroMode);

Vec3 accScaled{scaleLsbToPhysical(s.accX, accScale),
               scaleLsbToPhysical(s.accY, accScale),
               scaleLsbToPhysical(s.accZ, accScale)};
Vec3 accPhysical = applyCalibration(accCalib, accScaled); // m/s^2

Vec3 gyroScaled{scaleLsbToPhysical(s.gyroX, gyroScale),
                scaleLsbToPhysical(s.gyroY, gyroScale),
                scaleLsbToPhysical(s.gyroZ, gyroScale)};
Vec3 gyroPhysical = applyCalibration(gyroCalib, gyroScaled); // deg/s
```

## Examples

| File                              | What it demonstrates                                                                 |
|------------------------------------|----------------------------------------------------------------------------------------|
| `examples/example_of_use.cpp`      | The full manual sequence: `Init` → `GetSerial`/`GetSensorPosition`/`GetAvailableMeasurementModes` → `GetScaleFactor`/`GetCalibrationData` (Acc/Gyro/Mag) → `SetMeasurementMode` → `PrepareMeasurement` → `StartMeasurement` → live `RawData14` stream with calibration applied → `StopMeasurement`. |
| `examples/full_command_test.cpp`   | The remaining commands: `KeepAlive`, `SensorStatsExt`, `MeasurementActive`, `UnPrepareMeasurement`, and a `RawData15` stream. |
| `examples/framing_selftest.cpp`    | Verifies `encodeFrame()`/`FrameDecoder` byte-for-byte against the manual's worked stuffing example (`SetSerial` with serial `240`). |

Run any of them with the serial device as the first argument, e.g.:

```sh
./build/example_of_use /dev/ttyACM0 5       # stream for 5 seconds
./build/full_command_test /dev/ttyACM0
./build/framing_selftest
```


## Notes
- This code is fully written and tested by AI