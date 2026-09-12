#pragma once

#include "loco_sim/can_frame.h"

#include <cstdint>
#include <vector>

namespace loco_sim {

// Host implementation of the SerialCAN USB framing used by python_monitor.
//
// Layout:
//   0xFF 0xFF | size | id(4 BE) | uuid(2 BE) | data[size] | xor-checksum | "LOCO\r\n"
//
// Checksum is XOR of every byte from the first start marker through the last
// data byte (not including the checksum or trailer).
std::vector<uint8_t> encode_serialcan(const CanFrame& frame);

// Parse a complete buffer. Returns true and fills `out` on success.
bool decode_serialcan(const uint8_t* bytes, size_t length, CanFrame& out, size_t* consumed = nullptr);

class SerialCanParser {
public:
    void push(uint8_t byte);
    void push(const uint8_t* bytes, size_t n);
    bool pop(CanFrame& out);
    void reset();

private:
    std::vector<uint8_t> buf_;
};

}  // namespace loco_sim
