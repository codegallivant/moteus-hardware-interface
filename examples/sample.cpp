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

    MoteusAPI::Controller* controller1 = MoteusAPI::Controller::create(1, "can0");
    MoteusAPI::Controller* controller2 = MoteusAPI::Controller::create(2, "can0");

    // Specify which parameters should be read
    MoteusAPI::ReadState rs({
        .d_current = true,
        .q_current = true,
        .torque = true,
        .velocity = true
    });

    // Read operation on controller 1
    controller1->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();

    // Read operation on controller 2
    controller2->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();


    // Sending constant velocity command for 5 seconds to both controllers
    auto start_time = std::chrono::high_resolution_clock::now();
    int elapsed_time_count = 0;
    while(elapsed_time_count <= 5000) {
        
        MoteusAPI::CommandState cs({
            .kp_scale = 0.1,
            .kd_scale = 0.1,
            .position = std::numeric_limits<double>::quiet_NaN(),
            .velocity = 2.0,
        });
        
        controller1->write(cs);
        controller2->write(cs);

        // Can also write and fetch read response
        // controller->write(cs, rs);

        auto current_time = std::chrono::high_resolution_clock::now();
        elapsed_time_count += std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    }
}