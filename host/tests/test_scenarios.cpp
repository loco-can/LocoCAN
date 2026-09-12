#include "test_support.h"

#include "loco_sim/scenarios.h"

TEST(scenario_drive) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("drive", false);
    CHECK(result.passed);
}

TEST(scenario_heartbeat_loss) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("heartbeat-loss", false);
    CHECK(result.passed);
}

TEST(scenario_dual_controller) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("dual-controller", false);
    CHECK(result.passed);
}

TEST(scenario_lights) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("lights", false);
    CHECK(result.passed);
}

TEST(scenario_setup) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("setup", false);
    CHECK(result.passed);
}

TEST(scenario_signal) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("signal", false);
    CHECK(result.passed);
}

TEST(scenario_all) {
    loco_sim::ScenarioResult result = loco_sim::run_scenario("all", false);
    CHECK(result.passed);
}
