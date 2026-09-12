#include "loco_sim/scenarios.h"

#include "can_protocol.h"
#include "loco_sim/modules.h"
#include "loco_sim/protocol.h"
#include "loco_sim/virtual_bus.h"

#include <iostream>
#include <sstream>

namespace loco_sim {
namespace {

struct Harness {
    SimClock clock;
    VirtualBus bus;
    ControllerNode controller;
    VehicleNode vehicle;
    LightNode lights;

    Harness()
        : bus(clock),
          controller(0xA001, "controller"),
          vehicle(0xB002, "vehicle"),
          lights(0xC003, "lights") {
        bus.attach(controller);
        bus.attach(vehicle);
        bus.attach(lights);
    }

    void drive_on(uint16_t throttle) {
        controller.set_mains(true);
        controller.set_drive(true);
        controller.set_throttle(throttle);
        controller.set_power(900);
        bus.tick_all();
        bus.advance_and_tick(250);
    }
};

void log_frames(const VirtualBus& bus, std::ostream& out) {
    for (const CanFrame& frame : bus.log()) {
        out << "  " << describe(frame) << "\n";
    }
}

ScenarioResult fail(const std::string& name, const std::string& detail) {
    return ScenarioResult{name, false, detail};
}

ScenarioResult ok(const std::string& name, const std::string& detail) {
    return ScenarioResult{name, true, detail};
}

ScenarioResult scenario_drive(bool verbose) {
    Harness h;
    h.drive_on(400);

    std::ostringstream detail;
    if (verbose) {
        log_frames(h.bus, detail);
    }

    if (!h.controller.mains()) {
        return fail("drive", "controller mains did not stay on");
    }
    if (!h.vehicle.ready() || !h.vehicle.moving()) {
        return fail("drive", "vehicle did not become ready and moving");
    }
    if (h.vehicle.speed() != 400) {
        return fail("drive", "vehicle speed did not follow throttle");
    }
    if (!h.bus.saw_id(CAN_ID_DRIVE) || !h.bus.saw_id(CAN_ID_DRIVE_HEARTBEAT) ||
        !h.bus.saw_id(CAN_ID_VEHICLE_STATUS) || !h.bus.saw_id(CAN_ID_SPEED)) {
        return fail("drive", "expected drive/heartbeat/status/speed frames missing");
    }
    detail << "vehicle ready, moving, speed=" << h.vehicle.speed();
    return ok("drive", detail.str());
}

ScenarioResult scenario_heartbeat_loss(bool verbose) {
    Harness h;
    h.drive_on(512);
    h.controller.set_online(false);
    h.bus.advance_and_tick(MODULE_HEARTBEAT_TIMEOUT + 20);

    std::ostringstream detail;
    if (verbose) {
        log_frames(h.bus, detail);
    }

    if (!h.vehicle.emergency_stopped() || !h.vehicle.error() || h.vehicle.moving()) {
        return fail("heartbeat-loss", "vehicle did not emergency-stop after heartbeat timeout");
    }
    detail << "vehicle e-stop after " << MODULE_HEARTBEAT_TIMEOUT << "ms without heartbeat";
    return ok("heartbeat-loss", detail.str());
}

ScenarioResult scenario_dual_controller(bool verbose) {
    SimClock clock;
    VirtualBus bus(clock);
    ControllerNode a(0xA001, "controller-a");
    ControllerNode b(0xA004, "controller-b");
    VehicleNode vehicle(0xB002, "vehicle");

    bus.attach(a);
    bus.attach(b);
    bus.attach(vehicle);

    a.set_mains(true);
    a.set_drive(true);
    a.set_throttle(300);
    bus.tick_all();
    bus.advance_and_tick(120);

    b.set_mains(true);
    bus.tick_all();
    bus.advance_and_tick(80);

    std::ostringstream detail;
    if (verbose) {
        log_frames(bus, detail);
    }

    if (!a.mains() || a.locked()) {
        return fail("dual-controller", "first controller lost the bus");
    }
    if (!b.locked() || b.mains()) {
        return fail("dual-controller", "second controller was not locked out");
    }
    detail << "controller-a active, controller-b locked";
    return ok("dual-controller", detail.str());
}

ScenarioResult scenario_lights(bool verbose) {
    Harness h;
    h.controller.set_mains(true);
    LightCommand cmd;
    cmd.main = true;
    cmd.train = true;
    cmd.cab = true;
    h.controller.set_lights(cmd);
    h.bus.tick_all();
    h.bus.advance_and_tick(250);

    std::ostringstream detail;
    if (verbose) {
        log_frames(h.bus, detail);
    }

    if (!h.lights.lights().train || !h.lights.lights().cab) {
        return fail("lights", "light module did not apply train/cab flags");
    }
    if (!h.bus.saw_id(CAN_ID_LIGHT) || !h.bus.saw_id(CAN_ID_LIGHT_CURRENT)) {
        return fail("lights", "missing LIGHT or LIGHT_CURRENT frames");
    }
    detail << "lights current=" << h.lights.current() << " mA-sim";
    return ok("lights", detail.str());
}

ScenarioResult scenario_setup(bool verbose) {
    Harness h;
    h.bus.send(nullptr, make_setup_request());

    std::ostringstream detail;
    if (verbose) {
        log_frames(h.bus, detail);
    }

    bool replied = false;
    for (const CanFrame& frame : h.bus.log()) {
        if ((frame.id & CAN_SETUP_MASK) == CAN_ID_SETUP || frame.id == CAN_ID_REPLY) {
            if (frame.uuid == h.vehicle.uuid() && frame.size >= 7) {
                replied = true;
            }
        }
    }
    // Reply id is CAN_ID_REPLY | 0 = 0x780
    if (!h.bus.saw_id_from(CAN_ID_REPLY, h.vehicle.uuid())) {
        return fail("setup", "vehicle did not reply to setup request");
    }
    (void)replied;
    detail << "vehicle replied to 0x7FF setup request";
    return ok("setup", detail.str());
}

ScenarioResult scenario_signal(bool verbose) {
    Harness h;
    h.controller.set_mains(true);
    SignalCommand horn;
    horn.high = true;
    h.controller.set_signal(horn);
    h.bus.tick_all();
    h.bus.advance_and_tick(50);

    std::ostringstream detail;
    if (verbose) {
        log_frames(h.bus, detail);
    }
    if (!h.bus.saw_id(CAN_ID_SIGNAL)) {
        return fail("signal", "no SIGNAL frame on the bus");
    }
    const CanFrame frame = h.bus.frames_with_id(CAN_ID_SIGNAL).back();
    if (!SignalCommand::from_frame(frame).high) {
        return fail("signal", "SIGNAL_HIGH was not set");
    }
    detail << "horn high sent";
    return ok("signal", detail.str());
}

}  // namespace

TrainRig attach_default_train(VirtualBus& bus, ControllerNode& controller, VehicleNode& vehicle,
                              LightNode& lights) {
    bus.attach(controller);
    bus.attach(vehicle);
    bus.attach(lights);
    return TrainRig{&controller, &vehicle, &lights};
}

std::vector<std::string> scenario_names() {
    return {"drive", "heartbeat-loss", "dual-controller", "lights", "setup", "signal"};
}

ScenarioResult run_scenario(const std::string& name, bool verbose) {
    if (name == "drive") {
        return scenario_drive(verbose);
    }
    if (name == "heartbeat-loss") {
        return scenario_heartbeat_loss(verbose);
    }
    if (name == "dual-controller") {
        return scenario_dual_controller(verbose);
    }
    if (name == "lights") {
        return scenario_lights(verbose);
    }
    if (name == "setup") {
        return scenario_setup(verbose);
    }
    if (name == "signal") {
        return scenario_signal(verbose);
    }
    if (name == "all") {
        std::ostringstream detail;
        bool passed = true;
        for (const std::string& item : scenario_names()) {
            ScenarioResult result = run_scenario(item, verbose);
            detail << (result.passed ? "PASS " : "FAIL ") << result.name << ": " << result.detail << "\n";
            passed = passed && result.passed;
        }
        return ScenarioResult{"all", passed, detail.str()};
    }
    return fail(name, "unknown scenario");
}

}  // namespace loco_sim
