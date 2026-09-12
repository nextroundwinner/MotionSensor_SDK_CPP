// Byte-stuffing, checksum and frame (de)serialization for the MotionSensor
// serial protocol (chapter "Structure of packages").
#pragma once

#include "motionsensor/Protocol.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace motionsensor {

// A single decoded frame (command + payload), with framing bytes and
// checksum already removed/verified.
struct Frame {
    uint16_t command = 0;
    std::vector<uint8_t> data;
};

// Checksum over not-stuffed command+data bytes (little endian sum, chapter
// "Checksum").
uint16_t computeChecksum(const uint8_t* commandAndData, size_t length);

// Builds a complete, ready-to-transmit frame (start byte .. stop byte,
// including stuffing) for the given command and payload.
std::vector<uint8_t> encodeFrame(uint16_t command, const std::vector<uint8_t>& payload = {});

// Incremental frame decoder. Feed raw bytes as they arrive from the serial
// port; whenever a complete, checksum-valid frame has been received the
// onFrame callback is invoked. Frames with an invalid checksum are silently
// dropped (mirrors how a corrupted byte stream should be handled: wait for
// the next start byte).
class FrameDecoder {
public:
    using FrameCallback = std::function<void(const Frame&)>;

    explicit FrameDecoder(FrameCallback onFrame) : onFrame_(std::move(onFrame)) {}

    void feed(const uint8_t* data, size_t length);
    void feed(uint8_t byte);

private:
    enum class State { WaitStart, InFrame, InEscape };

    void handleCompleteFrame();

    State state_ = State::WaitStart;
    std::vector<uint8_t> buffer_; // unstuffed checksum(2) + command(2) + data(N)
    FrameCallback onFrame_;
};

} // namespace motionsensor
