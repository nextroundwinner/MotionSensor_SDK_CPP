// Minimal raw-mode serial port wrapper for Linux (termios), used to talk to
// the MotionSensor over its USB-CDC / Bluetooth-SPP virtual COM port.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace motionsensor {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens the given device (e.g. "/dev/ttyACM0") in raw mode, 8N1, no flow
    // control. baudRate must be a rate understood by termios (e.g. 460800 or
    // 115200, see chapter "Serial connection").
    // Throws std::runtime_error on failure.
    void open(const std::string& device, int baudRate);

    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Writes all bytes, blocking until done. Throws std::runtime_error on
    // failure.
    void write(const std::vector<uint8_t>& data);

    // Reads up to bufferSize bytes into buffer, waiting at most timeoutMs
    // milliseconds for at least one byte to arrive. Returns the number of
    // bytes actually read (0 on timeout). Throws std::runtime_error on
    // failure (other than timeout).
    size_t read(uint8_t* buffer, size_t bufferSize, int timeoutMs);

private:
    int fd_ = -1;
};

} // namespace motionsensor
