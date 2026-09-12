#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace loco_sim {

// Host-side copy of the firmware CAN_MESSAGE layout, plus a timestamp
// used by the virtual bus. The 11-bit message type lives in `id`; the
// 18-bit board UUID lives in `uuid`. Hardware packs both into a 29-bit
// extended identifier: (id << 18) | (uuid & 0x3FFFF).
struct CanFrame {
    uint32_t id = 0;
    uint32_t uuid = 0;
    std::array<uint8_t, 8> data{};
    uint8_t size = 0;
    uint64_t timestamp_ms = 0;

    uint32_t extended_id() const;
    static CanFrame from_extended(uint32_t ext_id, const uint8_t* payload, uint8_t n);
};

uint32_t pack_extended_id(uint32_t std_id, uint32_t uuid);
void unpack_extended_id(uint32_t ext_id, uint32_t& std_id, uint32_t& uuid);

std::string hex_bytes(const uint8_t* data, uint8_t size);
std::string frame_to_string(const CanFrame& frame);

}  // namespace loco_sim
