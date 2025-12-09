#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <csignal>
#include "api/moteus_driver_controller.hpp"
#include "lib/cxxopts.hpp"


void signalHandler(int signum) {
    MoteusDriverController::destroyAll();
}

int main(int argc, char** argv) {

    std::signal(SIGINT, signalHandler);

    cxxopts::Options options("MoteusRead", "Read Moteus Controller State");
    options.allow_unrecognised_options();

    options.add_options()
        // Reading options
        ("p,position", "Read Position")
        ("v,velocity", "Read Velocity")
        ("t,torque", "Read Torque")
        ("temperature", "Read Board Temperature")
        ("abs-position", "Read Absolute Position")
        ("aux1-gpio", "Read Aux1 GPIO")
        ("aux2-gpio", "Read Aux2 GPIO")
        ("d-current", "Read D Current")
        ("q-current", "Read Q Current")
        ("fault", "Read Fault Status")
        ("home-state", "Read Home State")
        ("mode", "Read Mode")
        ("motor-temperature", "Read Motor Temperature")
        ("power", "Read Power")
        ("trajectory-complete", "Read Trajectory Complete Flag")
        ("voltage", "Read Voltage")

        // transport args
        ("moteus-id", "Moteus ID", cxxopts::value<int>()->default_value("1"))
        ("socketcan-iface", "SocketCAN Interface", cxxopts::value<std::string>()->default_value("can0"))
        ("socketcan-ignore-errors", "Ignore SocketCAN errors", cxxopts::value<int>()->default_value("0"))
        ("can-disable-brs", "Disable CAN BRS (bit-rate switching)", cxxopts::value<int>()->default_value("0"))

        // Timing (Keep as int)
        ("duration-ms", "Duration in milliseconds", cxxopts::value<int>())

        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    int moteus_id = result["moteus-id"].as<int>();
    std::string socketcan_iface = result["socketcan-iface"].as<std::string>();
    int socketcan_ignore_errors = result["socketcan-ignore-errors"].as<int>();
    int socketcan_disable_brs = result["can-disable-brs"].as<int>();

    MoteusDriverController* mc = MoteusDriverController::create(
        moteus_id,
        socketcan_iface,
        socketcan_ignore_errors,
        socketcan_disable_brs
    );

    MoteusDriverController::motor_state readState;

    auto check_and_add = [&](const std::string& flag, const std::string& key) {
        if (result.count(flag)) {
            readState[key] = std::nullopt;
        }
    };

    check_and_add("position",           "position");
    check_and_add("velocity",           "velocity");
    check_and_add("torque",             "torque");
    check_and_add("temperature",        "temperature");
    check_and_add("abs-position",       "abs_position");
    check_and_add("aux1-gpio",          "aux1_gpio");
    check_and_add("aux2-gpio",          "aux2_gpio");
    check_and_add("d-current",          "d_current");
    check_and_add("q-current",          "q_current");
    check_and_add("fault",              "fault");
    check_and_add("home-state",         "home_state");
    check_and_add("mode",               "mode");
    check_and_add("motor-temperature",  "motor_temperature");
    check_and_add("power",              "power");
    check_and_add("trajectory-complete","trajectory_complete");
    check_and_add("voltage",            "voltage");

    if (readState.empty()) {
        readState["position"] = std::nullopt;
        // readState["velocity"] = std::nullopt;
        // readState["torque"] = std::nullopt;
        // readState["temperature"] = std::nullopt;
        // readState["voltage"] = std::nullopt;
    }

    std::cout << "Sending query..." << std::endl;

    if(result.count("duration-ms")) {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool status = false;
        int count = 0;
        int elapsed_time_count = 0;
        while (true) {
            if (elapsed_time_count >= result["duration-ms"].as<int>()) {
                break;
            }
            auto start_time = std::chrono::high_resolution_clock::now();
            status = mc->read(readState);
            auto end_time = std::chrono::high_resolution_clock::now();
            if(status) {
                count++;
            } else {
                return 0;
            }

            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            elapsed_time_count += elapsed_time.count();
        }
        std::cout <<  "Frequency (successful reads per second): " << count/(static_cast<double>(elapsed_time_count)/1000) << std::endl;
    }
    bool status = mc->read(readState);
    if(status) {
        MoteusDriverController::displayState(readState);
    } else {
        std::cerr << "Failed to read" << std::endl;
        return 1;
    }

    return 0;
}
