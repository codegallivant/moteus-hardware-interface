// Tests velocity control
#include "moteus/api/moteus_api.hpp" // Make sure to have the relevant moteus_api.hpp, moteus_api.cpp and moteus drivers present at appropriate paths
#include <csignal>
#include <chrono>


void signal_handler(int signal) {
    // Cleanly stops controllers and releases memory
    MoteusAPI::Controller::destroyAll();
}

int main() {
    // Important to ensure all servos are stopped upon any termination signal 
    std::signal(SIGINT, signal_handler);

    MoteusAPI::Controller* controller = MoteusAPI::Controller::create(1, "can0");

    // Specify which parameters should be read
    MoteusAPI::ReadState rs({
        .velocity = true,
    });

    // Velocity control
    MoteusAPI::CommandState cs({
        .kp_scale = 0.1,
        .kd_scale = 0.1,
        .position = std::numeric_limits<double>::quiet_NaN(),
        .velocity = 2.0,
    });
    int duration_ms = 20000;
    auto start_time = std::chrono::high_resolution_clock::now();
    int count = 0;
    int elapsed = 0;
    while (elapsed < duration_ms) {
        auto t0 = std::chrono::high_resolution_clock::now();
        bool read_status = controller->write(cs, rs);
        auto t1 = std::chrono::high_resolution_clock::now();
        count++;
        elapsed += std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        if(rs.getValues()["velocity"]->has_value() && read_status) {
            std::cout << "Read velocity: " << rs.getValues()["velocity"]->value() << std::endl;
        } else {
            std::cout << "Could not read velocity" << std::endl;
        }
    }
    std::cout << "Control Frequency (writes/s): "
              << count / (elapsed / 1000.0) << std::endl;
}