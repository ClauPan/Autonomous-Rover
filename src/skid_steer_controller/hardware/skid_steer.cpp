// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "skid_steer_controller/skid_steer.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace skid_steer_controller {

hardware_interface::CallbackReturn SkidSteerControllerHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    front_left_wheel_joint = info_.hardware_parameters["front_left_wheel_joint"];
    back_left_wheel_joint = info_.hardware_parameters["back_left_wheel_joint"];
    front_right_wheel_joint = info_.hardware_parameters["front_right_wheel_joint"];
    back_right_wheel_joint = info_.hardware_parameters["back_right_wheel_joint"];

    int enc_counts_per_rev_fw = std::stoi(info_.hardware_parameters["enc_counts_per_rev_fw"]);
    int enc_counts_per_rev_ba = std::stoi(info_.hardware_parameters["enc_counts_per_rev_ba"]);
    wheel_fl_.setup((2*M_PI)/enc_counts_per_rev_fw, (2*M_PI)/enc_counts_per_rev_ba);
    wheel_bl_.setup((2*M_PI)/enc_counts_per_rev_fw, (2*M_PI)/enc_counts_per_rev_ba);
    wheel_fr_.setup((2*M_PI)/enc_counts_per_rev_fw, (2*M_PI)/enc_counts_per_rev_ba);
    wheel_br_.setup((2*M_PI)/enc_counts_per_rev_fw, (2*M_PI)/enc_counts_per_rev_ba);

    vel_mult_linear = std::stof(info_.hardware_parameters["vel_mult_linear"]);
    vel_mult_angular = std::stof(info_.hardware_parameters["vel_mult_angular"]);
    
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("SkidSteerControllerHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());

            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("SkidSteerControllerHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
 
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("SkidSteerControllerHardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());

            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("SkidSteerControllerHardware"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("SkidSteerControllerHardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SkidSteerControllerHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        front_left_wheel_joint, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        front_left_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        back_left_wheel_joint, hardware_interface::HW_IF_POSITION, &wheel_bl_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        back_left_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_bl_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        front_right_wheel_joint, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        front_right_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        back_right_wheel_joint, hardware_interface::HW_IF_POSITION, &wheel_br_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        back_right_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_br_.vel));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SkidSteerControllerHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        front_left_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        back_left_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_bl_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        front_right_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        back_right_wheel_joint, hardware_interface::HW_IF_VELOCITY, &wheel_br_.cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn SkidSteerControllerHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Configuring ...please wait...");
    if (arduino.connected()) {
        arduino.disconnect();
    }
    arduino.connect();
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SkidSteerControllerHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Cleaning up ...please wait...");
    if (arduino.connected()) {
        arduino.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SkidSteerControllerHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) 
{
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Activating ...please wait...");
    if (!arduino.connected()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    arduino.send("u 20:12:0:50\r");
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SkidSteerControllerHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("SkidSteerControllerHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SkidSteerControllerHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) 
{
    if (!arduino.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    double dt = period.seconds();
    
    arduino.read(wheel_fl_.enc, wheel_fr_.enc);

    double pos_prev = wheel_fl_.pos;
    wheel_fl_.pos = wheel_fl_.get_encoder_angle();
    wheel_fl_.vel = (wheel_fl_.pos - pos_prev) / dt;

    wheel_bl_.pos = wheel_fl_.pos;
    wheel_bl_.vel = wheel_fl_.vel;

    pos_prev = wheel_fr_.pos;
    wheel_fr_.pos = wheel_fr_.get_encoder_angle();
    wheel_fr_.vel = (wheel_fr_.pos - pos_prev) / dt;

    wheel_br_.pos = wheel_fr_.pos;
    wheel_br_.vel = wheel_fr_.vel;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::SkidSteerControllerHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
    
    if (!arduino.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    int motor_l = wheel_fl_.get_counts_per_loop();
    int motor_r = wheel_fr_.get_counts_per_loop();

    if (motor_l == motor_r) {
        motor_l *= vel_mult_linear;
        motor_r *= vel_mult_linear;

    }
    else if (motor_l_counts_per_loop == -motor_r_counts_per_loop) {
        motor_l *= vel_mult_angular;
        motor_r *= vel_mult_angular;
    }

    arduino.set_motors(motor_l, motor_r);

    return hardware_interface::return_type::OK;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(skid_steer_controller::SkidSteerControllerHardware, hardware_interface::SystemInterface)
