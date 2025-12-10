#pragma once
#include "moteus_drivers/moteus.h"
#include <unordered_map>
#include <memory>
#include <iostream>
#include <string>
#include <optional>


namespace MoteusAPI {

    typedef struct CommandStateParams {
        std::optional<double> accel_limit = std::nullopt;
        std::optional<double> feedforward_torque = std::nullopt;
        std::optional<double> ilimit_scale = std::nullopt;
        std::optional<double> kp_scale = std::nullopt;
        std::optional<double> kd_scale = std::nullopt;
        std::optional<double> maximum_torque = std::nullopt;
        std::optional<double> position = std::nullopt;
        std::optional<double> stop_position = std::nullopt;
        std::optional<double> watchdog_timeout = std::nullopt;
        std::optional<double> velocity = std::nullopt;
        std::optional<double> velocity_limit = std::nullopt;
    } CommandStateParams;


    typedef struct ReadStateParams {
        bool abs_position = false;
        bool aux1_gpio = false;
        bool aux2_gpio = false;
        bool d_current = false;
        bool fault = false;
        bool home_state = false;
        bool mode = false;
        bool motor_temperature = false;
        bool position = true;
        bool power = false;
        bool q_current = false;
        bool temperature = false;
        bool torque = false;
        bool trajectory_complete = false;
        bool velocity = false;
        bool voltage = false;
    } ReadStateParams;

    template<typename T>
    void displayState(std::unordered_map<std::string, T*> m) {
        for (const auto& [key, opt_val] : m) {
            std::cout << key << ": ";
            if (opt_val && opt_val->has_value()) {
                std::cout << opt_val->value();
            } else {
                std::cout << "null";
            }
            std::cout << "\n";
        }
    }

    class CommandState {
    public:
        // Values
        const std::optional<double> accel_limit;    
        const std::optional<double> feedforward_torque;
        const std::optional<double> ilimit_scale;
        const std::optional<double> kp_scale;
        const std::optional<double> kd_scale;
        const std::optional<double> maximum_torque;
        const std::optional<double> position;
        const std::optional<double> stop_position;
        const std::optional<double> watchdog_timeout;
        const std::optional<double> velocity;
        const std::optional<double> velocity_limit;
        
        // Maps
        std::unordered_map<std::string, const std::optional<double>*> valueMap;

        // Moteus Driver Resolution/Format
        mjbots::moteus::PositionMode::Format format; // Format in which command is given (i.e. to write in)
        // Moteus Driver Command
        mjbots::moteus::PositionMode::Command command;

        CommandState(const CommandStateParams& p)
            : accel_limit(p.accel_limit),
            feedforward_torque(p.feedforward_torque),
            ilimit_scale(p.ilimit_scale),
            kp_scale(p.kp_scale),
            kd_scale(p.kd_scale),
            maximum_torque(p.maximum_torque),
            position(p.position),
            stop_position(p.stop_position),
            watchdog_timeout(p.watchdog_timeout),
            velocity(p.velocity),
            velocity_limit(p.velocity_limit)
        {
            createMaps();
            this->calculateFormat();
            this->calculateCommand();
        }

        std::unordered_map<std::string, const std::optional<double>*> getValues() const {
            return valueMap;
        }

        void display() const {
            displayState<const std::optional<double>>(this->getValues());
        }

        private:
        // Maps
        std::unordered_map<std::string, mjbots::moteus::Resolution*> formatMap;
        std::unordered_map<std::string, double*> commandMap;

        void createMaps() {
            valueMap = {
                {"accel_limit", &accel_limit},
                {"feedforward_torque", &feedforward_torque},
                {"ilimit_scale", &ilimit_scale},
                {"kp_scale", &kp_scale},
                {"kd_scale", &kd_scale},
                {"maximum_torque", &maximum_torque},
                {"position", &position},
                {"stop_position", &stop_position},
                {"watchdog_timeout", &watchdog_timeout},
                {"velocity", &velocity},
                {"velocity_limit", &velocity_limit}
            };

            formatMap = {
                {"accel_limit", &format.accel_limit},
                {"feedforward_torque", &format.feedforward_torque},
                {"ilimit_scale", &format.ilimit_scale},
                {"kp_scale", &format.kp_scale},
                {"kd_scale", &format.kd_scale},
                {"maximum_torque", &format.maximum_torque},
                {"position", &format.position},
                {"stop_position", &format.stop_position},
                {"watchdog_timeout", &format.watchdog_timeout},
                {"velocity", &format.velocity},
                {"velocity_limit", &format.velocity_limit}   
            };

            commandMap = {
                {"accel_limit", &command.accel_limit},
                {"feedforward_torque", &command.feedforward_torque},
                {"ilimit_scale", &command.ilimit_scale},
                {"kp_scale", &command.kp_scale},
                {"kd_scale", &command.kd_scale},
                {"maximum_torque", &command.maximum_torque},
                {"position", &command.position},
                {"stop_position", &command.stop_position},
                {"watchdog_timeout", &command.watchdog_timeout},
                {"velocity", &command.velocity},
                {"velocity_limit", &command.velocity_limit}   
            };
        }

        void calculateFormat() {
            std::optional<double> value;
            for(auto it = formatMap.begin() ; it != formatMap.end() ; it++) {
                value = *(valueMap[it->first]); 
                if (value.has_value()) {
                    *(it->second) = mjbots::moteus::Resolution::kFloat;
                } else {
                    *(it->second) = mjbots::moteus::Resolution::kIgnore;
                }
            }
        }

        void calculateCommand() {
            std::optional<double> value;
            for(auto it = commandMap.begin() ; it != commandMap.end() ; it++) {
                value = *(valueMap[it->first]); 
                if (value.has_value()) {
                    *(it->second) = value.value();
                }
            }
        }
    };


    class ReadState {
    public:
        typedef struct Param {
            std::optional<double> value; // Requested value
            const bool flag; // Request flag
            explicit Param(bool enabled = false) : flag(enabled) {}
        } Param;

        // Parameters
        Param abs_position;
        Param aux1_gpio;
        Param aux2_gpio;
        Param d_current;
        Param fault; // int
        Param home_state; // int
        Param mode; // int
        Param motor_temperature;
        Param position;
        Param power;
        Param q_current;
        Param temperature;
        Param torque;
        Param trajectory_complete; // int
        Param velocity;
        Param voltage;

        //Maps
        std::unordered_map<std::string, std::optional<double>*> valueMap;
        std::unordered_map<std::string, Param*> paramMap;

        // Moteus Driver Resolution/Format
        mjbots::moteus::Query::Format format;

        ReadState(const ReadStateParams& p)
            : abs_position(p.abs_position)
            , aux1_gpio(p.aux1_gpio)
            , aux2_gpio(p.aux2_gpio)
            , d_current(p.d_current)
            , fault(p.fault)
            , home_state(p.home_state)
            , mode(p.mode)
            , motor_temperature(p.motor_temperature)
            , position(p.position)
            , power(p.power)
            , q_current(p.q_current)
            , temperature(p.temperature)
            , torque(p.torque)
            , trajectory_complete(p.trajectory_complete)
            , velocity(p.velocity)
            , voltage(p.voltage)
        {
            this->createMaps();
            this->calculateFormat();
        }

        std::unordered_map<std::string, std::optional<double>*> getValues() {
            return valueMap;
        }

        void display() {
            displayState<std::optional<double>>(this->getValues());
        }

        void populateState(mjbots::moteus::Query::Result values) {
            std::unordered_map<std::string, std::optional<double>> resultMap = {
                {"abs_position", values.abs_position},
                {"aux1_gpio", values.aux1_gpio},
                {"aux2_gpio", values.aux2_gpio},
                {"d_current", values.d_current},
                {"fault", values.fault},
                {"home_state", static_cast<float>(values.home_state)},
                {"mode", static_cast<float>(values.mode)},
                {"motor_temperature", values.motor_temperature},
                {"position", values.position},
                {"power", values.power},
                {"q_current", values.q_current},
                {"temperature", values.temperature},
                {"torque", values.torque},
                {"trajectory_complete", values.trajectory_complete},
                {"velocity", values.velocity},
                {"voltage", values.voltage}
            };

            for(auto it = resultMap.begin() ; it != resultMap.end() ; it++) {
                if(paramMap[it->first]->flag) {
                    paramMap[it->first]->value = resultMap[it->first];
                }
            }
        }

    private:
        // Maps
        std::unordered_map<std::string, mjbots::moteus::Resolution*> formatMap;
        std::unordered_map<std::string, mjbots::moteus::Resolution> formatDefaultsMap;

        void createMaps() {
            paramMap = {
                {"abs_position", &abs_position},
                {"aux1_gpio", &aux1_gpio},
                {"aux2_gpio", &aux2_gpio},
                {"d_current", &d_current},
                {"fault", &fault},
                {"home_state", &home_state},
                {"mode", &mode},
                {"motor_temperature", &motor_temperature},
                {"position", &position},
                {"power", &power},
                {"q_current", &q_current},
                {"temperature", &temperature},
                {"torque", &torque},
                {"trajectory_complete", &trajectory_complete},
                {"velocity", &velocity},
                {"voltage", &voltage}
            };

            valueMap = {
                {"abs_position", &abs_position.value},
                {"aux1_gpio", &aux1_gpio.value},
                {"aux2_gpio", &aux2_gpio.value},
                {"d_current", &d_current.value},
                {"fault", &fault.value},
                {"home_state", &home_state.value},
                {"mode", &mode.value},
                {"motor_temperature", &motor_temperature.value},
                {"position", &position.value},
                {"power", &power.value},
                {"q_current", &q_current.value},
                {"temperature", &temperature.value},
                {"torque", &torque.value},
                {"trajectory_complete", &trajectory_complete.value},
                {"velocity", &velocity.value},
                {"voltage", &voltage.value}
            };

            formatMap = {
                {"abs_position", &format.abs_position},
                {"aux1_gpio", &format.aux1_gpio},
                {"aux2_gpio", &format.aux2_gpio},
                {"d_current", &format.d_current},
                {"fault", &format.fault},
                {"home_state", &format.home_state},
                {"mode", &format.mode},
                {"motor_temperature", &format.motor_temperature},
                {"position", &format.position},
                {"power", &format.power},
                {"q_current", &format.q_current},
                {"temperature", &format.temperature},
                {"torque", &format.torque},
                {"trajectory_complete", &format.trajectory_complete},
                {"velocity", &format.velocity},
                {"voltage", &format.voltage}
            };

            formatDefaultsMap = {
                {"abs_position", mjbots::moteus::Resolution::kFloat},
                {"aux1_gpio", mjbots::moteus::Resolution::kFloat},
                {"aux2_gpio", mjbots::moteus::Resolution::kFloat},
                {"d_current", mjbots::moteus::Resolution::kFloat},
                {"fault", mjbots::moteus::Resolution::kInt8},
                {"home_state", mjbots::moteus::Resolution::kInt16},
                {"mode", mjbots::moteus::Resolution::kInt8},
                {"motor_temperature", mjbots::moteus::Resolution::kFloat},
                {"position", mjbots::moteus::Resolution::kFloat},
                {"power", mjbots::moteus::Resolution::kFloat},
                {"q_current", mjbots::moteus::Resolution::kFloat},
                {"temperature", mjbots::moteus::Resolution::kFloat},
                {"torque", mjbots::moteus::Resolution::kFloat},
                {"trajectory_complete", mjbots::moteus::Resolution::kInt16},
                {"velocity", mjbots::moteus::Resolution::kFloat},
                {"voltage", mjbots::moteus::Resolution::kFloat}
            };
        }

        void calculateFormat() {
            for(auto it = formatMap.begin() ; it != formatMap.end() ; it++) {
                Param param = *(paramMap[it->first]); 
                if(param.flag) {
                    *(it->second) = formatDefaultsMap[it->first];
                } else {
                    *(it->second) = mjbots::moteus::Resolution::kIgnore;
                }
            }
        }
    };


    class Controller {
    public:
        inline static std::vector<std::unique_ptr<MoteusAPI::Controller>> registry{};

        std::unique_ptr<mjbots::moteus::Controller> internal_controller;

        Controller(int moteus_id = 1, std::string socketcan_iface = "can0", int socketcan_ignore_errors = 0, int socketcan_disable_brs = 0) {
            mjbots::moteus::Controller::Options options;
            options.id = moteus_id;
            options.default_query = false; // Whether to get read response after writing for every command

            std::vector<std::string> transport_args = {
                "--socketcan-iface", socketcan_iface,
                "--socketcan-ignore-errors", std::to_string(socketcan_ignore_errors),
                "--can-disable-brs", std::to_string(socketcan_disable_brs)
            };
            auto transport = mjbots::moteus::Controller::MakeSingletonTransport(transport_args);
            options.transport = transport;
            
            // options.position_format.kp_scale = mjbots::moteus::kFloat;
            // options.position_format.kd_scale = mjbots::moteus::kFloat;
            // options.position_format.ilimit_scale = mjbots::moteus::kFloat;
            // options.position_format.feedforward_torque = mjbots::moteus::kFloat;

            this->internal_controller = std::make_unique<mjbots::moteus::Controller>(options);
            this->internal_controller->SetStop();
        }

        ~Controller() {
            terminate();
        }

        void terminate() {
            this->internal_controller->SetStop();
        }

        // Use if opting for centralized memory control for all initialised controllers
        static MoteusAPI::Controller* create(
            int moteus_id = 1,
            std::string socketcan_iface = "can0",
            int socketcan_ignore_errors = 0,
            int socketcan_disable_brs = 0)
        {
            MoteusAPI::Controller::registry.push_back(std::make_unique<MoteusAPI::Controller>(
                moteus_id,
                socketcan_iface,
                socketcan_ignore_errors,
                socketcan_disable_brs
            ));
            return registry.back().get();
        }

        // Only use if controllers have been created with create()
        static void destroyAll() {
            std::cout << "Resetting all Moteus Driver Controllers" << std::endl;
            MoteusAPI::Controller::registry.clear();
        }

        bool read(ReadState& rs) {
            const auto result = this->internal_controller->SetQuery(&rs.format);
            if(!result) {
                return false;
            }
            rs.populateState(result->values);
            return true;
        }

        bool write(const CommandState& cs, ReadState& rs) { 
            auto result = this->internal_controller->SetPosition(cs.command, &cs.format, &rs.format);  
            if(!result) {
                return false;
            }
            rs.populateState(result->values);
            return true;
        }
        
        void write(const CommandState& cs) {
            auto result = this->internal_controller->SetPosition(cs.command, &cs.format);  
        }

        void writeDuration(const CommandState& cs, int duration_ms) {
            auto start_time = std::chrono::high_resolution_clock::now();
            bool status;
            int count = 0;
            int elapsed_time_count = 0;
            while (true) {
                if (elapsed_time_count >= duration_ms) {
                    break;
                }
                auto start_time = std::chrono::high_resolution_clock::now();
                this->write(cs);
                auto end_time = std::chrono::high_resolution_clock::now();
                count++;
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                elapsed_time_count += elapsed_time.count();
            }
            std::cout <<  "Control Frequency (successful writes per second): " << count/(static_cast<double>(elapsed_time_count)/1000) << std::endl;
        }
    };

}