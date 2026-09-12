#include "loco_sim/protocol.h"

#include "can_protocol.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace loco_sim {
namespace {

CanFrame make_frame(uint32_t id, uint32_t uuid, const uint8_t* data, uint8_t size) {
    CanFrame frame;
    frame.id = id;
    frame.uuid = uuid;
    frame.size = size;
    for (uint8_t i = 0; i < size && i < 8; ++i) {
        frame.data[i] = data[i];
    }
    return frame;
}

uint16_t word_be(uint8_t msb, uint8_t lsb) {
    return static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

}  // namespace

std::string id_name(uint32_t id) {
    switch (id) {
        case CAN_ID_DRIVE_HEARTBEAT:
            return "DRIVE_HEARTBEAT";
        case CAN_ID_TRAINEND_HEARTBEAT:
            return "TRAINEND_HEARTBEAT";
        case CAN_ID_CURRENT:
            return "CURRENT";
        case CAN_ID_MOTOR_CURRENT:
            return "MOTOR_CURRENT";
        case CAN_ID_BATT_CURRENT:
            return "BATT_CURRENT";
        case CAN_ID_LIGHT_CURRENT:
            return "LIGHT_CURRENT";
        case CAN_ID_SPEED:
            return "SPEED";
        case CAN_ID_DIR:
            return "DIR";
        case CAN_ID_SIGNAL:
            return "SIGNAL";
        case CAN_ID_TACHO:
            return "TACHO";
        case CAN_ID_VOLTAGE:
            return "VOLTAGE";
        case CAN_ID_MOTOR_VOLTAGE:
            return "MOTOR_VOLTAGE";
        case CAN_ID_BATT_VOLTAGE:
            return "BATT_VOLTAGE";
        case CAN_ID_BATT_1_VOLTAGE:
            return "BATT_1_VOLTAGE";
        case CAN_ID_BATT_2_VOLTAGE:
            return "BATT_2_VOLTAGE";
        case CAN_ID_DRIVE:
            return "DRIVE";
        case CAN_ID_LIGHT:
            return "LIGHT";
        case CAN_ID_SWITCH:
            return "SWITCH";
        case CAN_ID_VEHICLE_STATUS:
            return "VEHICLE_STATUS";
        case CAN_ID_MODULE_HEARTBEAT:
            return "MODULE_HEARTBEAT";
        case CAN_ID_LOCO_SETUP:
            return "LOCO_SETUP";
        case CAN_ID_REQUEST:
            return "SETUP_REQUEST";
        default:
            break;
    }
    if ((id & CAN_SETUP_MASK) == CAN_ID_SETUP) {
        return "SETUP";
    }
    if ((id & CAN_REPLY_MASK) == CAN_ID_REPLY || (id & 0x780) == CAN_ID_REPLY) {
        return "SETUP_REPLY";
    }
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << id;
    return out.str();
}

std::string describe(const CanFrame& frame) {
    std::ostringstream out;
    out << "t=" << frame.timestamp_ms << "ms " << id_name(frame.id) << " " << frame_to_string(frame);
    if (frame.id == CAN_ID_DRIVE && frame.size >= 8) {
        DriveCommand cmd = DriveCommand::from_frame(frame);
        out << " mains=" << cmd.mains << " drive=" << cmd.drive << " dir=" << cmd.dir
            << " thr=" << cmd.throttle << " pwr=" << cmd.power << " brk=" << cmd.brake
            << " multi=" << cmd.multi_traction;
    }
    if (frame.id == CAN_ID_VEHICLE_STATUS && frame.size >= 3) {
        VehicleStatus st = VehicleStatus::from_frame(frame);
        out << " mains=" << st.mains << " ready=" << st.ready << " moving=" << st.moving
            << " err=" << st.error;
    }
    if (frame.id == CAN_ID_LIGHT && frame.size >= 1) {
        LightCommand light = LightCommand::from_frame(frame);
        out << " main=" << light.main << " train=" << light.train << " cab=" << light.cab
            << " high=" << light.high;
    }
    return out.str();
}

CanFrame DriveCommand::to_frame(uint32_t sender_uuid) const {
    uint8_t payload[8] = {0};

    payload[0] = set_flag(payload[0], CONTROL_ERROR_FLAG, error);
    payload[0] = set_flag(payload[0], CONTROL_AUX_1_FLAG, aux1);
    payload[0] = set_flag(payload[0], CONTROL_AUX_FLAG, aux);
    payload[0] = set_flag(payload[0], CONTROL_DOWN_FLAG, down);
    payload[0] = set_flag(payload[0], CONTROL_UP_FLAG, up);
    payload[0] = set_flag(payload[0], CONTROL_DIR_FLAG, dir);
    payload[0] = set_flag(payload[0], CONTROL_DRIVE_FLAG, drive);
    payload[0] = set_flag(payload[0], CONTROL_MAINS_FLAG, mains);

    const uint16_t uuid_bits = master_uuid;
    const uint16_t drive_word = static_cast<uint16_t>(
        (throttle & 0x3FFu) | (((uuid_bits >> 11) & 0x1Fu) << 10) | (multi_traction ? 0x8000u : 0u));
    const uint16_t power_word =
        static_cast<uint16_t>((power & 0x3FFu) | (((uuid_bits >> 6) & 0x1Fu) << 10));
    const uint16_t brake_word = static_cast<uint16_t>((brake & 0x3FFu) | ((uuid_bits & 0x3Fu) << 10));

    payload[2] = static_cast<uint8_t>(drive_word >> 8);
    payload[3] = static_cast<uint8_t>(drive_word & 0xFFu);
    payload[4] = static_cast<uint8_t>(power_word >> 8);
    payload[5] = static_cast<uint8_t>(power_word & 0xFFu);
    payload[6] = static_cast<uint8_t>(brake_word >> 8);
    payload[7] = static_cast<uint8_t>(brake_word & 0xFFu);

    return make_frame(CAN_ID_DRIVE, sender_uuid, payload, 8);
}

DriveCommand DriveCommand::from_frame(const CanFrame& frame) {
    DriveCommand cmd;
    if (frame.size < 1) {
        return cmd;
    }
    const uint8_t flags = frame.data[0];
    cmd.error = flag_set(flags, CONTROL_ERROR_FLAG);
    cmd.aux1 = flag_set(flags, CONTROL_AUX_1_FLAG);
    cmd.aux = flag_set(flags, CONTROL_AUX_FLAG);
    cmd.down = flag_set(flags, CONTROL_DOWN_FLAG);
    cmd.up = flag_set(flags, CONTROL_UP_FLAG);
    cmd.dir = flag_set(flags, CONTROL_DIR_FLAG);
    cmd.drive = flag_set(flags, CONTROL_DRIVE_FLAG);
    cmd.mains = flag_set(flags, CONTROL_MAINS_FLAG);

    if (frame.size >= 8) {
        const uint16_t drive_word = word_be(frame.data[2], frame.data[3]);
        const uint16_t power_word = word_be(frame.data[4], frame.data[5]);
        const uint16_t brake_word = word_be(frame.data[6], frame.data[7]);
        cmd.throttle = drive_word & 0x3FFu;
        cmd.power = power_word & 0x3FFu;
        cmd.brake = brake_word & 0x3FFu;
        cmd.multi_traction = (drive_word & 0x8000u) != 0;
        cmd.master_uuid = static_cast<uint16_t>(((drive_word >> 10) & 0x1Fu) << 11 |
                                                ((power_word >> 10) & 0x1Fu) << 6 |
                                                ((brake_word >> 10) & 0x3Fu));
    }
    return cmd;
}

CanFrame VehicleStatus::to_frame(uint32_t sender_uuid) const {
    uint8_t payload[3] = {0};
    payload[0] = set_flag(payload[0], ERROR_FLAG, error);
    payload[0] = set_flag(payload[0], READY_FLAG, ready);
    payload[0] = set_flag(payload[0], MOVING_FLAG, moving);
    payload[0] = set_flag(payload[0], MULTI_FLAG, multi);
    payload[0] = set_flag(payload[0], REVERSE, reverse);
    payload[0] = set_flag(payload[0], DIR_FLAG, dir);
    payload[0] = set_flag(payload[0], DRIVE_FLAG, drive);
    payload[0] = set_flag(payload[0], MAINS_FLAG, mains);
    const uint16_t reported = uuid != 0 ? uuid : static_cast<uint16_t>(sender_uuid);
    payload[1] = static_cast<uint8_t>(reported & 0xFFu);
    payload[2] = static_cast<uint8_t>((reported >> 8) & 0xFFu);
    return make_frame(CAN_ID_VEHICLE_STATUS, sender_uuid, payload, 3);
}

VehicleStatus VehicleStatus::from_frame(const CanFrame& frame) {
    VehicleStatus st;
    if (frame.size < 1) {
        return st;
    }
    const uint8_t flags = frame.data[0];
    st.error = flag_set(flags, ERROR_FLAG);
    st.ready = flag_set(flags, READY_FLAG);
    st.moving = flag_set(flags, MOVING_FLAG);
    st.multi = flag_set(flags, MULTI_FLAG);
    st.reverse = flag_set(flags, REVERSE);
    st.dir = flag_set(flags, DIR_FLAG);
    st.drive = flag_set(flags, DRIVE_FLAG);
    st.mains = flag_set(flags, MAINS_FLAG);
    if (frame.size >= 3) {
        st.uuid = static_cast<uint16_t>(frame.data[1] | (static_cast<uint16_t>(frame.data[2]) << 8));
    }
    return st;
}

uint8_t LightCommand::to_byte() const {
    uint8_t bits = 0;
    bits = set_flag(bits, LIGHT_MAIN, main);
    bits = set_flag(bits, LIGHT_TRAIN, train);
    bits = set_flag(bits, LIGHT_CAB, cab);
    bits = set_flag(bits, LIGHT_INSTR, instr);
    bits = set_flag(bits, LIGHT_BACK, back);
    bits = set_flag(bits, LIGHT_HIGH, high);
    bits = set_flag(bits, LIGHT_LOW, low);
    bits = set_flag(bits, LIGHT_POSIT, posit);
    return bits;
}

LightCommand LightCommand::from_byte(uint8_t bits) {
    LightCommand cmd;
    cmd.main = flag_set(bits, LIGHT_MAIN);
    cmd.train = flag_set(bits, LIGHT_TRAIN);
    cmd.cab = flag_set(bits, LIGHT_CAB);
    cmd.instr = flag_set(bits, LIGHT_INSTR);
    cmd.back = flag_set(bits, LIGHT_BACK);
    cmd.high = flag_set(bits, LIGHT_HIGH);
    cmd.low = flag_set(bits, LIGHT_LOW);
    cmd.posit = flag_set(bits, LIGHT_POSIT);
    return cmd;
}

CanFrame LightCommand::to_frame(uint32_t sender_uuid) const {
    uint8_t payload[1] = {to_byte()};
    return make_frame(CAN_ID_LIGHT, sender_uuid, payload, 1);
}

LightCommand LightCommand::from_frame(const CanFrame& frame) {
    if (frame.size < 1) {
        return LightCommand{};
    }
    return from_byte(frame.data[0]);
}

uint8_t SignalCommand::to_byte() const {
    uint8_t bits = 0;
    bits = set_flag(bits, SIGNAL_HIGH, high);
    bits = set_flag(bits, SIGNAL_LOW, low);
    bits = set_flag(bits, SIGNAL_BELL, bell);
    return bits;
}

SignalCommand SignalCommand::from_byte(uint8_t bits) {
    SignalCommand cmd;
    cmd.high = flag_set(bits, SIGNAL_HIGH);
    cmd.low = flag_set(bits, SIGNAL_LOW);
    cmd.bell = flag_set(bits, SIGNAL_BELL);
    return cmd;
}

CanFrame SignalCommand::to_frame(uint32_t sender_uuid) const {
    uint8_t payload[1] = {to_byte()};
    return make_frame(CAN_ID_SIGNAL, sender_uuid, payload, 1);
}

SignalCommand SignalCommand::from_frame(const CanFrame& frame) {
    if (frame.size < 1) {
        return SignalCommand{};
    }
    return from_byte(frame.data[0]);
}

CanFrame LocoSetup::to_frame(uint32_t sender_uuid) const {
    uint8_t payload[3] = {0};
    payload[0] = set_flag(payload[0], 0, status);
    payload[0] = set_flag(payload[0], 1, disable);
    payload[1] = static_cast<uint8_t>(target_uuid & 0xFFu);
    payload[2] = static_cast<uint8_t>((target_uuid >> 8) & 0xFFu);
    return make_frame(CAN_ID_LOCO_SETUP, sender_uuid, payload, 3);
}

LocoSetup LocoSetup::from_frame(const CanFrame& frame) {
    LocoSetup setup;
    if (frame.size < 3) {
        return setup;
    }
    setup.status = flag_set(frame.data[0], 0);
    setup.disable = flag_set(frame.data[0], 1);
    setup.target_uuid = static_cast<uint16_t>(frame.data[1] | (static_cast<uint16_t>(frame.data[2]) << 8));
    return setup;
}

CanFrame make_heartbeat(uint32_t id, uint32_t sender_uuid) {
    return make_frame(id, sender_uuid, nullptr, 0);
}

CanFrame make_u16_telemetry(uint32_t id, uint32_t sender_uuid, uint16_t value) {
    uint8_t payload[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFFu)};
    return make_frame(id, sender_uuid, payload, 2);
}

uint16_t read_u16_telemetry(const CanFrame& frame) {
    if (frame.size < 2) {
        return 0;
    }
    return word_be(frame.data[0], frame.data[1]);
}

CanFrame make_setup_request() {
    CanFrame frame;
    frame.id = CAN_ID_REQUEST;
    frame.size = 0;
    return frame;
}

CanFrame make_setup_reply(uint32_t sender_uuid, uint8_t data_id, const std::string& text) {
    uint8_t payload[8] = {0};
    const size_t n = std::min(text.size(), static_cast<size_t>(8));
    for (size_t i = 0; i < n; ++i) {
        payload[i] = static_cast<uint8_t>(text[i]);
    }
    return make_frame(CAN_ID_REPLY | (data_id & 0x7Fu), sender_uuid, payload, static_cast<uint8_t>(n));
}

}  // namespace loco_sim
