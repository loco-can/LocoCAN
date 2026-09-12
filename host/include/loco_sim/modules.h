#pragma once

#include "loco_sim/protocol.h"
#include "loco_sim/virtual_bus.h"

#include <cstdint>
#include <limits>
#include <string>

namespace loco_sim {

// Virtual stand-in for a controller module. Sends drive heartbeat and
// drive packets on the same intervals as the firmware (50 ms / 100 ms).
class ControllerNode : public Node {
public:
    explicit ControllerNode(uint32_t uuid, std::string name = "controller");

    std::string name() const override { return name_; }
    uint32_t uuid() const override { return uuid_; }
    void on_rx(const CanFrame& frame) override;
    void tick(uint64_t now_ms) override;

    void set_mains(bool on);
    void set_online(bool on);
    bool online() const { return online_; }
    void set_drive(bool on);
    void set_dir(bool reverse);
    void set_throttle(uint16_t value);
    void set_power(uint16_t value);
    void set_brake(uint16_t value);
    void set_lights(const LightCommand& lights);
    void set_signal(const SignalCommand& signal);
    void set_multi(bool on, uint16_t master_uuid);

    bool mains() const { return drive_.mains; }
    bool locked() const { return locked_; }
    bool error() const { return drive_.error; }
    const DriveCommand& drive() const { return drive_; }
    const LightCommand& lights() const { return lights_; }

private:
    void send_drive();
    void send_heartbeat();
    void send_lights();
    void send_signal();

    uint32_t uuid_;
    std::string name_;
    DriveCommand drive_;
    LightCommand lights_;
    SignalCommand signal_;
    bool online_ = true;
    bool locked_ = false;
    bool pending_lights_ = false;
    bool pending_signal_ = false;
    uint64_t last_heartbeat_ms_ = std::numeric_limits<uint64_t>::max();
    uint64_t last_drive_ms_ = std::numeric_limits<uint64_t>::max();
    uint64_t seen_foreign_heartbeat_ms_ = std::numeric_limits<uint64_t>::max();
};

// Virtual stand-in for a motor / vehicle module. Follows drive commands
// while the controller heartbeat is alive; emergency-stops on timeout.
class VehicleNode : public Node {
public:
    explicit VehicleNode(uint32_t uuid, std::string name = "vehicle");

    std::string name() const override { return name_; }
    uint32_t uuid() const override { return uuid_; }
    void on_rx(const CanFrame& frame) override;
    void tick(uint64_t now_ms) override;

    bool mains() const { return status_.mains; }
    bool ready() const { return status_.ready; }
    bool moving() const { return status_.moving; }
    bool error() const { return status_.error; }
    bool emergency_stopped() const { return emergency_stopped_; }
    uint16_t speed() const { return speed_; }
    uint16_t motor_current() const { return motor_current_; }
    const DriveCommand& last_drive() const { return last_drive_; }
    const VehicleStatus& status() const { return status_; }

private:
    void apply_drive(const DriveCommand& cmd, uint32_t sender);
    void emergency_stop();
    void send_status();
    void send_telemetry();

    uint32_t uuid_;
    std::string name_;
    DriveCommand last_drive_;
    VehicleStatus status_;
    bool have_drive_ = false;
    bool emergency_stopped_ = false;
    uint16_t speed_ = 0;
    uint16_t motor_current_ = 0;
    uint16_t voltage_ = 240;  // 24.0 V in tenths
    uint64_t last_heartbeat_ms_ = std::numeric_limits<uint64_t>::max();
    uint64_t last_status_ms_ = std::numeric_limits<uint64_t>::max();
    uint64_t last_telem_ms_ = std::numeric_limits<uint64_t>::max();
};

// Virtual stand-in for a switch / light module.
class LightNode : public Node {
public:
    explicit LightNode(uint32_t uuid, std::string name = "lights");

    std::string name() const override { return name_; }
    uint32_t uuid() const override { return uuid_; }
    void on_rx(const CanFrame& frame) override;
    void tick(uint64_t now_ms) override;

    LightCommand lights() const { return lights_; }
    uint16_t current() const { return current_; }

private:
    uint32_t uuid_;
    std::string name_;
    LightCommand lights_;
    uint16_t current_ = 0;
    uint64_t last_telem_ms_ = std::numeric_limits<uint64_t>::max();
};

}  // namespace loco_sim
