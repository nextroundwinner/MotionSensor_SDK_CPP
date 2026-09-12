// Verifies encodeFrame()/FrameDecoder against the worked example from the
// manual (chapter "Structure of packages", "Stuffing example"):
//   SetSerial command with serial number decimal 240 (hex 0xF0)
//   Pre-stuff:  F0 30 01 40 00 F0 00 00 00 0F
//   Post-stuff: F0 30 01 40 00 81 A5 00 00 00 0F
#include "motionsensor/Framing.h"
#include "motionsensor/Protocol.h"

#include <cstdio>
#include <vector>

using namespace motionsensor;

int main() {
    std::vector<uint8_t> payload;
    uint32_t serial = 240;
    payload.push_back(static_cast<uint8_t>(serial & 0xFF));
    payload.push_back(static_cast<uint8_t>((serial >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>((serial >> 16) & 0xFF));
    payload.push_back(static_cast<uint8_t>((serial >> 24) & 0xFF));

    auto frame = encodeFrame(static_cast<uint16_t>(Command::SetSerial), payload);

    std::vector<uint8_t> expected = {0xF0, 0x30, 0x01, 0x40, 0x00, 0x81, 0xA5, 0x00, 0x00, 0x00, 0x0F};

    std::printf("Encoded: ");
    for (uint8_t b : frame) std::printf("%02X ", b);
    std::printf("\n");

    bool ok = (frame == expected);
    std::printf("Matches manual example: %s\n", ok ? "YES" : "NO");
    if (!ok) return 1;

    // Round-trip through the decoder.
    bool decodedOk = false;
    Frame decoded;
    FrameDecoder decoder([&](const Frame& f) {
        decoded = f;
        decodedOk = true;
    });
    decoder.feed(frame.data(), frame.size());

    if (!decodedOk) {
        std::printf("Decoder did not produce a frame!\n");
        return 1;
    }
    bool cmdOk = decoded.command == static_cast<uint16_t>(Command::SetSerial);
    bool dataOk = decoded.data == payload;
    std::printf("Decoded command matches: %s\n", cmdOk ? "YES" : "NO");
    std::printf("Decoded payload matches: %s\n", dataOk ? "YES" : "NO");

    return (cmdOk && dataOk) ? 0 : 1;
}
