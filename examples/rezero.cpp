// Tests setting zero offset
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
        .position = true,
    });

    // Velocity control
    MoteusAPI::CommandState cs({
        .kp_scale = 0.1,
        .kd_scale = 0.1,
        .position = std::numeric_limits<double>::quiet_NaN(),
        .velocity = 2.0,
    });
    controller->writeDuration(cs, 2000);

    // Read operation on controller
    controller->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();

    std::cout << std::endl;

    // Rezero
    std::cout << "Setting zero offset.." << std::endl;
    controller->setZeroOffset();

    std::cout << std::endl;

    // Read operation on controller
    controller->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();
}