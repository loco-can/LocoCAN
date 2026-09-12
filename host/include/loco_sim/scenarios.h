#pragma once

#include <string>
#include <vector>

namespace loco_sim {

class VirtualBus;
class ControllerNode;
class VehicleNode;
class LightNode;

struct ScenarioResult {
    std::string name;
    bool passed = false;
    std::string detail;
};

struct TrainRig {
    ControllerNode* controller;
    VehicleNode* vehicle;
    LightNode* lights;
};

TrainRig attach_default_train(VirtualBus& bus, ControllerNode& controller, VehicleNode& vehicle,
                              LightNode& lights);

// Built-in end-to-end scenarios that exercise the protocol without hardware.
ScenarioResult run_scenario(const std::string& name, bool verbose);
std::vector<std::string> scenario_names();

}  // namespace loco_sim
