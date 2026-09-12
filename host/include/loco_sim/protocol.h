#pragma once

#include "loco_sim/can_frame.h"

#include <cstdint>
#include <string>

namespace loco_sim {

inline bool flag_set(uint8_t bits, unsigned pos) {
    return (bits & (1u << pos)) != 0;
}

inline uint8_t set_flag(uint8_t bits, unsigned pos, bool value) {
    if (value) {
        return static_cast<uint8_t>(bits | (1u << pos));
    }
    return static_cast<uint8_t>(bits & ~(1u << pos));
}

std::string id_name(uint32_t id);
std::string describe(const CanFrame& frame);

struct DriveCommand {
    bool error = false;
    bool aux1 = false;
    bool aux = false;
    bool down = false;
    bool up = false;
    bool dir = false;    // 0 = forward, 1 = reverse
    bool drive = false;
    bool mains = false;
    uint16_t throttle = 0;  // 10-bit
    uint16_t power = 0;     // 10-bit
    uint16_t brake = 0;     // 10-bit
    bool multi_traction = false;
    uint16_t master_uuid = 0;

    CanFrame to_frame(uint32_t sender_uuid) const;
    static DriveCommand from_frame(const CanFrame& frame);
};

struct VehicleStatus {
    bool error = false;
    bool ready = false;
    bool moving = false;
    bool multi = false;
    bool reverse = false;
    bool dir = false;
    bool drive = false;
    bool mains = false;
    uint16_t uuid = 0;

    CanFrame to_frame(uint32_t sender_uuid) const;
    static VehicleStatus from_frame(const CanFrame& frame);
};

struct LightCommand {
    bool main = false;
    bool train = false;
    bool cab = false;
    bool instr = false;
    bool back = false;
    bool high = false;
    bool low = false;
    bool posit = false;

    uint8_t to_byte() const;
    static LightCommand from_byte(uint8_t bits);
    CanFrame to_frame(uint32_t sender_uuid) const;
    static LightCommand from_frame(const CanFrame& frame);
};

struct SignalCommand {
    bool high = false;
    bool low = false;
    bool bell = false;

    uint8_t to_byte() const;
    static SignalCommand from_byte(uint8_t bits);
    CanFrame to_frame(uint32_t sender_uuid) const;
    static SignalCommand from_frame(const CanFrame& frame);
};

struct LocoSetup {
    bool status = false;   // 0 = select master, 1 = reverse direction
    bool disable = false;
    uint16_t target_uuid = 0;

    CanFrame to_frame(uint32_t sender_uuid) const;
    static LocoSetup from_frame(const CanFrame& frame);
};

CanFrame make_heartbeat(uint32_t id, uint32_t sender_uuid);
CanFrame make_u16_telemetry(uint32_t id, uint32_t sender_uuid, uint16_t value);
uint16_t read_u16_telemetry(const CanFrame& frame);

CanFrame make_setup_request();
CanFrame make_setup_reply(uint32_t sender_uuid, uint8_t data_id, const std::string& text);

}  // namespace loco_sim
