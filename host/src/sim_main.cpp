#include "loco_sim/modules.h"
#include "loco_sim/protocol.h"
#include "loco_sim/scenarios.h"
#include "loco_sim/virtual_bus.h"

#include "can_protocol.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> split(const std::string& line) {
    std::istringstream in(line);
    std::vector<std::string> parts;
    std::string part;
    while (in >> part) {
        parts.push_back(part);
    }
    return parts;
}

void print_help() {
    std::cout
        << "Loco-CAN host simulator (no Arduino hardware required)\n"
        << "\n"
        << "Usage:\n"
        << "  loco-sim                         interactive REPL\n"
        << "  loco-sim --scenario NAME         run a canned scenario\n"
        << "  loco-sim --scenario all          run every scenario\n"
        << "  loco-sim --list                  list scenarios\n"
        << "  loco-sim --help\n"
        << "\n"
        << "REPL commands:\n"
        << "  mains on|off       throttle <0-1023>   power <0-1023>   brake <0-1023>\n"
        << "  drive on|off       dir forward|reverse   horn high|low|off\n"
        << "  lights <csv>       csv flags: main,train,cab,instr,back,high,low,posit\n"
        << "  tick <ms>          log [id]            status            setup\n"
        << "  drop               reconnect           scenario <name>   help   quit\n";
}

loco_sim::LightCommand parse_lights(const std::string& csv) {
    loco_sim::LightCommand cmd;
    std::string token;
    std::istringstream in(csv);
    while (std::getline(in, token, ',')) {
        if (token == "main") cmd.main = true;
        else if (token == "train") cmd.train = true;
        else if (token == "cab") cmd.cab = true;
        else if (token == "instr") cmd.instr = true;
        else if (token == "back") cmd.back = true;
        else if (token == "high") cmd.high = true;
        else if (token == "low") cmd.low = true;
        else if (token == "posit") cmd.posit = true;
    }
    return cmd;
}

void print_status(const loco_sim::ControllerNode& c, const loco_sim::VehicleNode& v,
                  const loco_sim::LightNode& l, const loco_sim::SimClock& clock) {
    std::cout << "t=" << clock.now_ms() << "ms\n";
    std::cout << "  controller uuid=0x" << std::hex << c.uuid() << std::dec
              << " mains=" << c.mains() << " locked=" << c.locked()
              << " throttle=" << c.drive().throttle << " online=" << c.online() << "\n";
    std::cout << "  vehicle    uuid=0x" << std::hex << v.uuid() << std::dec
              << " ready=" << v.ready() << " moving=" << v.moving()
              << " speed=" << v.speed() << " e-stop=" << v.emergency_stopped() << "\n";
    std::cout << "  lights     train=" << l.lights().train << " cab=" << l.lights().cab
              << " current=" << l.current() << "\n";
}

int run_repl() {
    using namespace loco_sim;
    SimClock clock;
    VirtualBus bus(clock);
    ControllerNode controller(0xA001, "controller");
    VehicleNode vehicle(0xB002, "vehicle");
    LightNode lights(0xC003, "lights");
    attach_default_train(bus, controller, vehicle, lights);

    std::cout << "Loco-CAN virtual train. Type 'help'.\n";
    print_status(controller, vehicle, lights, clock);

    std::string line;
    while (std::cout << "sim> " && std::getline(std::cin, line)) {
        auto args = split(line);
        if (args.empty()) {
            continue;
        }
        const std::string& cmd = args[0];
        if (cmd == "quit" || cmd == "exit") {
            break;
        }
        if (cmd == "help") {
            print_help();
            continue;
        }
        if (cmd == "status") {
            print_status(controller, vehicle, lights, clock);
            continue;
        }
        if (cmd == "log") {
            uint32_t filter_id = 0;
            if (args.size() >= 2) {
                filter_id = static_cast<uint32_t>(std::stoul(args[1], nullptr, 0));
            }
            for (const CanFrame& frame : bus.log()) {
                if (filter_id == 0 || frame.id == filter_id) {
                    std::cout << describe(frame) << "\n";
                }
            }
            continue;
        }
        if (cmd == "tick" && args.size() >= 2) {
            const size_t before = bus.log().size();
            bus.advance_and_tick(static_cast<uint64_t>(std::stoul(args[1])));
            for (size_t i = before; i < bus.log().size(); ++i) {
                std::cout << describe(bus.log()[i]) << "\n";
            }
            print_status(controller, vehicle, lights, clock);
            continue;
        }
        if (cmd == "mains" && args.size() >= 2) {
            controller.set_mains(args[1] == "on");
            bus.tick_all();
            continue;
        }
        if (cmd == "drive" && args.size() >= 2) {
            controller.set_drive(args[1] == "on");
            continue;
        }
        if (cmd == "dir" && args.size() >= 2) {
            controller.set_dir(args[1] == "reverse");
            continue;
        }
        if (cmd == "throttle" && args.size() >= 2) {
            controller.set_throttle(static_cast<uint16_t>(std::stoul(args[1])));
            continue;
        }
        if (cmd == "power" && args.size() >= 2) {
            controller.set_power(static_cast<uint16_t>(std::stoul(args[1])));
            continue;
        }
        if (cmd == "brake" && args.size() >= 2) {
            controller.set_brake(static_cast<uint16_t>(std::stoul(args[1])));
            continue;
        }
        if (cmd == "lights" && args.size() >= 2) {
            controller.set_lights(parse_lights(args[1]));
            bus.tick_all();
            continue;
        }
        if (cmd == "horn" && args.size() >= 2) {
            SignalCommand signal;
            signal.high = args[1] == "high";
            signal.low = args[1] == "low";
            controller.set_signal(signal);
            bus.tick_all();
            continue;
        }
        if (cmd == "drop") {
            controller.set_online(false);
            std::cout << "controller disconnected from virtual bus\n";
            continue;
        }
        if (cmd == "reconnect") {
            controller.set_online(true);
            std::cout << "controller reconnected\n";
            continue;
        }
        if (cmd == "setup") {
            bus.send(nullptr, make_setup_request());
            for (const CanFrame& frame : bus.frames_with_id(CAN_ID_REPLY)) {
                std::cout << describe(frame) << "\n";
            }
            continue;
        }
        if (cmd == "scenario" && args.size() >= 2) {
            ScenarioResult result = run_scenario(args[1], true);
            std::cout << (result.passed ? "PASS" : "FAIL") << " " << result.name << "\n"
                      << result.detail << "\n";
            continue;
        }
        std::cout << "unknown command. Type help.\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace loco_sim;
    bool quiet = false;
    std::string scenario;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_help();
            return 0;
        }
        if (arg == "--list") {
            for (const std::string& name : scenario_names()) {
                std::cout << name << "\n";
            }
            return 0;
        }
        if (arg == "--quiet") {
            quiet = true;
            continue;
        }
        if ((arg == "--scenario" || arg == "-s") && i + 1 < argc) {
            scenario = argv[++i];
            continue;
        }
        if (arg.rfind("--scenario=", 0) == 0) {
            scenario = arg.substr(11);
            continue;
        }
        std::cerr << "unknown argument: " << arg << "\n";
        print_help();
        return 2;
    }

    if (!scenario.empty()) {
        ScenarioResult result = run_scenario(scenario, !quiet);
        if (!quiet) {
            std::cout << (result.passed ? "PASS" : "FAIL") << " " << result.name << "\n"
                      << result.detail << "\n";
        } else if (!result.passed) {
            std::cerr << "FAIL " << result.name << "\n" << result.detail << "\n";
        }
        return result.passed ? 0 : 1;
    }

    return run_repl();
}
