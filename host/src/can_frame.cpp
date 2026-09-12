#include "loco_sim/can_frame.h"

#include <iomanip>
#include <sstream>

namespace loco_sim {

uint32_t pack_extended_id(uint32_t std_id, uint32_t uuid) {
    return (std_id << 18) | (uuid & 0x3FFFFu);
}

void unpack_extended_id(uint32_t ext_id, uint32_t& std_id, uint32_t& uuid) {
    std_id = ext_id >> 18;
    uuid = ext_id & 0x3FFFFu;
}

uint32_t CanFrame::extended_id() const {
    return pack_extended_id(id, uuid);
}

CanFrame CanFrame::from_extended(uint32_t ext_id, const uint8_t* payload, uint8_t n) {
    CanFrame frame;
    unpack_extended_id(ext_id, frame.id, frame.uuid);
    frame.size = n > 8 ? 8 : n;
    if (payload != nullptr) {
        for (uint8_t i = 0; i < frame.size; ++i) {
            frame.data[i] = payload[i];
        }
    }
    return frame;
}

std::string hex_bytes(const uint8_t* data, uint8_t size) {
    std::ostringstream out;
    out << std::hex << std::uppercase << std::setfill('0');
    for (uint8_t i = 0; i < size; ++i) {
        if (i != 0) {
            out << ' ';
        }
        out << std::setw(2) << static_cast<int>(data[i]);
    }
    return out.str();
}

std::string frame_to_string(const CanFrame& frame) {
    std::ostringstream out;
    out << std::hex << std::uppercase << std::setfill('0');
    out << "id=0x" << std::setw(3) << frame.id;
    out << " uuid=0x" << std::setw(4) << frame.uuid;
    out << " ext=0x" << std::setw(8) << frame.extended_id();
    out << " [" << hex_bytes(frame.data.data(), frame.size) << "]";
    return out.str();
}

}  // namespace loco_sim
