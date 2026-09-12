#include "loco_sim/modules.h"

#include "can_protocol.h"

#include <limits>
#include <utility>

namespace loco_sim {
namespace {

constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
constexpr uint64_t kDriveHeartbeatMs = 50;
constexpr uint64_t kDrivePacketMs = 100;
constexpr uint64_t kStatusMs = 200;
constexpr uint64_t kTelemMs = 200;
constexpr uint64_t kModuleHeartbeatMs = 500;

uint16_t clamp10(uint16_t value) {
    return value > 0x3FFu ? 0x3FFu : value;
}

bool due(uint64_t last_ms, uint64_t now_ms, uint64_t interval_ms) {
    return last_ms == kNever || now_ms - last_ms >= interval_ms;
}

}  // namespace

ControllerNode::ControllerNode(uint32_t uuid, std::string name) : uuid_(uuid), name_(std::move(name)) {
    add_filter(CAN_ID_MASK, CAN_ID_DRIVE);
    add_filter(CAN_ID_MASK, CAN_ID_VEHICLE_STATUS);
    add_filter(CAN_ID_MASK, CAN_ID_DRIVE_HEARTBEAT);
    drive_.power = 1023;
}

void ControllerNode::on_rx(const CanFrame& frame) {
    if (frame.id == CAN_ID_DRIVE_HEARTBEAT && frame.uuid != uuid_) {
        seen_foreign_heartbeat_ms_ = frame.timestamp_ms;
        if (!drive_.mains) {
            locked_ = true;
        }
    }
    if (frame.id == CAN_ID_DRIVE && frame.uuid != uuid_ && !drive_.mains) {
        locked_ = true;
        drive_.error = true;
    }
}

void ControllerNode::tick(uint64_t now_ms) {
    if (!online_) {
        return;
    }
    if (locked_ && drive_.mains) {
        drive_.mains = false;
        drive_.drive = false;
        drive_.error = true;
    }

    if (drive_.mains && !locked_) {
        if (due(last_heartbeat_ms_, now_ms, kDriveHeartbeatMs)) {
            send_heartbeat();
            last_heartbeat_ms_ = now_ms;
        }
        if (due(last_drive_ms_, now_ms, kDrivePacketMs)) {
            send_drive();
            last_drive_ms_ = now_ms;
        }
        if (pending_lights_) {
            send_lights();
            pending_lights_ = false;
        }
        if (pending_signal_) {
            send_signal();
            pending_signal_ = false;
        }
    }
}

void ControllerNode::set_online(bool on) {
    online_ = on;
}

void ControllerNode::set_mains(bool on) {
    if (on) {
        const uint64_t now = bus() != nullptr ? bus()->clock().now_ms() : 0;
        if (locked_ || (seen_foreign_heartbeat_ms_ != kNever &&
                        now - seen_foreign_heartbeat_ms_ < MODULE_HEARTBEAT_TIMEOUT)) {
            locked_ = true;
            drive_.error = true;
            drive_.mains = false;
            return;
        }
        locked_ = false;
        drive_.error = false;
        drive_.mains = true;
        last_heartbeat_ms_ = kNever;
        last_drive_ms_ = kNever;
    } else {
        drive_.mains = false;
        drive_.drive = false;
    }
}

void ControllerNode::set_drive(bool on) {
    drive_.drive = on;
}

void ControllerNode::set_dir(bool reverse) {
    drive_.dir = reverse;
}

void ControllerNode::set_throttle(uint16_t value) {
    drive_.throttle = clamp10(value);
}

void ControllerNode::set_power(uint16_t value) {
    drive_.power = clamp10(value);
}

void ControllerNode::set_brake(uint16_t value) {
    drive_.brake = clamp10(value);
}

void ControllerNode::set_lights(const LightCommand& lights) {
    lights_ = lights;
    pending_lights_ = true;
}

void ControllerNode::set_signal(const SignalCommand& signal) {
    signal_ = signal;
    pending_signal_ = true;
}

void ControllerNode::set_multi(bool on, uint16_t master_uuid) {
    drive_.multi_traction = on;
    drive_.master_uuid = master_uuid;
}

void ControllerNode::send_drive() {
    send(drive_.to_frame(uuid_));
}

void ControllerNode::send_heartbeat() {
    send(make_heartbeat(CAN_ID_DRIVE_HEARTBEAT, uuid_));
}

void ControllerNode::send_lights() {
    send(lights_.to_frame(uuid_));
}

void ControllerNode::send_signal() {
    send(signal_.to_frame(uuid_));
}

VehicleNode::VehicleNode(uint32_t uuid, std::string name) : uuid_(uuid), name_(std::move(name)) {
    add_filter(CAN_ID_MASK, CAN_ID_DRIVE);
    add_filter(CAN_ID_MASK, CAN_ID_DRIVE_HEARTBEAT);
    add_filter(CAN_REQUEST_MASK, CAN_ID_REQUEST);
    add_filter(CAN_LOCO_SETUP_MASK, CAN_ID_LOCO_SETUP);
    status_.uuid = static_cast<uint16_t>(uuid_);
}

void VehicleNode::on_rx(const CanFrame& frame) {
    if (frame.id == CAN_ID_DRIVE_HEARTBEAT) {
        last_heartbeat_ms_ = frame.timestamp_ms;
        return;
    }
    if (frame.id == CAN_ID_DRIVE) {
        apply_drive(DriveCommand::from_frame(frame), frame.uuid);
        last_heartbeat_ms_ = frame.timestamp_ms;
        return;
    }
    if (frame.id == CAN_ID_REQUEST) {
        send(make_setup_reply(uuid_, 0, "VEHICLE"));
        return;
    }
    if (frame.id == CAN_ID_LOCO_SETUP) {
        LocoSetup setup = LocoSetup::from_frame(frame);
        if (setup.target_uuid == static_cast<uint16_t>(uuid_) && setup.disable) {
            emergency_stop();
        }
        if (setup.target_uuid == static_cast<uint16_t>(uuid_) && setup.status) {
            status_.reverse = !status_.reverse;
        }
    }
}

void VehicleNode::tick(uint64_t now_ms) {
    if (have_drive_ && status_.mains && last_heartbeat_ms_ != kNever &&
        now_ms - last_heartbeat_ms_ >= MODULE_HEARTBEAT_TIMEOUT) {
        emergency_stop();
    }

    if (status_.mains && !emergency_stopped_) {
        if (due(last_status_ms_, now_ms, kStatusMs)) {
            send_status();
            last_status_ms_ = now_ms;
        }
        if (due(last_telem_ms_, now_ms, kTelemMs)) {
            send_telemetry();
            last_telem_ms_ = now_ms;
        }
        if (now_ms % kModuleHeartbeatMs == 0) {
            send(make_heartbeat(CAN_ID_MODULE_HEARTBEAT, uuid_));
        }
    }
}

void VehicleNode::apply_drive(const DriveCommand& cmd, uint32_t /*sender*/) {
    last_drive_ = cmd;
    have_drive_ = true;
    if (emergency_stopped_ && cmd.mains && cmd.drive) {
        // Stay latched until mains is cycled off then on.
        if (!status_.mains && cmd.mains) {
            emergency_stopped_ = false;
            status_.error = false;
        }
    }
    if (!cmd.mains) {
        emergency_stopped_ = false;
        status_ = VehicleStatus{};
        status_.uuid = static_cast<uint16_t>(uuid_);
        speed_ = 0;
        motor_current_ = 0;
        return;
    }
    if (emergency_stopped_) {
        return;
    }

    status_.mains = true;
    status_.drive = cmd.drive;
    status_.dir = cmd.dir;
    status_.ready = true;
    status_.error = cmd.error;
    status_.multi = cmd.multi_traction;

    if (cmd.drive && cmd.brake == 0) {
        speed_ = cmd.throttle;
        motor_current_ = static_cast<uint16_t>(cmd.throttle / 2);
    } else {
        speed_ = 0;
        motor_current_ = 0;
    }
    status_.moving = speed_ > 0;
}

void VehicleNode::emergency_stop() {
    emergency_stopped_ = true;
    status_.error = true;
    status_.ready = false;
    status_.moving = false;
    status_.drive = false;
    status_.mains = false;
    speed_ = 0;
    motor_current_ = 0;
    send_status();
}

void VehicleNode::send_status() {
    send(status_.to_frame(uuid_));
}

void VehicleNode::send_telemetry() {
    send(make_u16_telemetry(CAN_ID_SPEED, uuid_, speed_));
    send(make_u16_telemetry(CAN_ID_MOTOR_CURRENT, uuid_, motor_current_));
    send(make_u16_telemetry(CAN_ID_VOLTAGE, uuid_, voltage_));
    send(make_u16_telemetry(CAN_ID_DIR, uuid_, status_.dir ? 1 : 0));
}

LightNode::LightNode(uint32_t uuid, std::string name) : uuid_(uuid), name_(std::move(name)) {
    add_filter(CAN_ID_MASK, CAN_ID_LIGHT);
    add_filter(CAN_ID_MASK, CAN_ID_DRIVE);
}

void LightNode::on_rx(const CanFrame& frame) {
    if (frame.id == CAN_ID_LIGHT) {
        lights_ = LightCommand::from_frame(frame);
        current_ = 0;
        if (lights_.main || lights_.train) {
            current_ += 80;
        }
        if (lights_.cab || lights_.instr) {
            current_ += 20;
        }
        if (lights_.high || lights_.low || lights_.posit || lights_.back) {
            current_ += 40;
        }
    }
    if (frame.id == CAN_ID_DRIVE) {
        DriveCommand cmd = DriveCommand::from_frame(frame);
        if (!cmd.mains) {
            lights_ = LightCommand{};
            current_ = 0;
        }
    }
}

void LightNode::tick(uint64_t now_ms) {
    if (current_ == 0) {
        return;
    }
    if (due(last_telem_ms_, now_ms, kTelemMs)) {
        send(make_u16_telemetry(CAN_ID_LIGHT_CURRENT, uuid_, current_));
        last_telem_ms_ = now_ms;
    }
}

}  // namespace loco_sim
