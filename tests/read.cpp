#include "api/moteus_driver_controller.hpp"


int main(int argc, char** argv) {
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);
    auto mc = std::make_unique<MoteusDriverController>();
    MoteusDriverController::motor_state readState;
    bool status = mc->read(readState);
    if(status) {
        MoteusDriverController::displayState(readState);
    } else {
        std::cout << "Failed to read" << std::endl;
    }
}