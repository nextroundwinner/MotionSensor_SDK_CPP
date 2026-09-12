#include "motionsensor/Framing.h"

namespace motionsensor {

namespace {
bool isSpecialByte(uint8_t b) {
    return b == framing::kStartByte || b == framing::kStopByte ||
           b == framing::kStuffByte || b == framing::kStuffKey;
}
} // namespace

uint16_t computeChecksum(const uint8_t* commandAndData, size_t length) {
    uint32_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += commandAndData[i];
    }
    return static_cast<uint16_t>(sum & 0xFFFF);
}

std::vector<uint8_t> encodeFrame(uint16_t command, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> commandAndData;
    commandAndData.reserve(2 + payload.size());
    commandAndData.push_back(static_cast<uint8_t>(command & 0xFF));
    commandAndData.push_back(static_cast<uint8_t>((command >> 8) & 0xFF));
    commandAndData.insert(commandAndData.end(), payload.begin(), payload.end());

    const uint16_t checksum = computeChecksum(commandAndData.data(), commandAndData.size());

    std::vector<uint8_t> unstuffed;
    unstuffed.reserve(2 + commandAndData.size());
    unstuffed.push_back(static_cast<uint8_t>(checksum & 0xFF));
    unstuffed.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF));
    unstuffed.insert(unstuffed.end(), commandAndData.begin(), commandAndData.end());

    std::vector<uint8_t> frame;
    frame.reserve(2 + unstuffed.size() * 2);
    frame.push_back(framing::kStartByte);
    for (uint8_t b : unstuffed) {
        if (isSpecialByte(b)) {
            frame.push_back(framing::kStuffByte);
            frame.push_back(static_cast<uint8_t>(b ^ framing::kStuffKey));
        } else {
            frame.push_back(b);
        }
    }
    frame.push_back(framing::kStopByte);
    return frame;
}

void FrameDecoder::feed(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        feed(data[i]);
    }
}

void FrameDecoder::feed(uint8_t byte) {
    switch (state_) {
        case State::WaitStart:
            if (byte == framing::kStartByte) {
                buffer_.clear();
                state_ = State::InFrame;
            }
            break;

        case State::InEscape:
            buffer_.push_back(static_cast<uint8_t>(byte ^ framing::kStuffKey));
            state_ = State::InFrame;
            break;

        case State::InFrame:
            if (byte == framing::kStuffByte) {
                state_ = State::InEscape;
            } else if (byte == framing::kStopByte) {
                handleCompleteFrame();
                state_ = State::WaitStart;
            } else if (byte == framing::kStartByte) {
                // Unexpected literal start byte inside a frame: resync.
                buffer_.clear();
            } else {
                buffer_.push_back(byte);
            }
            break;
    }
}

void FrameDecoder::handleCompleteFrame() {
    if (buffer_.size() < 4) {
        return; // too short to contain checksum + command
    }
    const uint16_t receivedChecksum =
        static_cast<uint16_t>(buffer_[0]) | (static_cast<uint16_t>(buffer_[1]) << 8);
    const uint8_t* commandAndData = buffer_.data() + 2;
    const size_t commandAndDataLen = buffer_.size() - 2;
    const uint16_t actualChecksum = computeChecksum(commandAndData, commandAndDataLen);
    if (receivedChecksum != actualChecksum) {
        return; // corrupted frame, drop it
    }

    Frame frame;
    frame.command = static_cast<uint16_t>(commandAndData[0]) |
                     (static_cast<uint16_t>(commandAndData[1]) << 8);
    frame.data.assign(commandAndData + 2, commandAndData + commandAndDataLen);
    if (onFrame_) {
        onFrame_(frame);
    }
}

} // namespace motionsensor
