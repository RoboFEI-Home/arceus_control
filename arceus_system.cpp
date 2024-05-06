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

#include "arceus_control/arceus_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arceus_omnidirectional_system
{
hardware_interface::CallbackReturn ArceusOmniSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.wheel1_name = info_.hardware_parameters["wheel1_name"];
  cfg_.wheel2_name = info_.hardware_parameters["wheel2_name"];
  cfg_.wheel3_name = info_.hardware_parameters["wheel3_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_pulses_per_rev = std::stoi(info_.hardware_parameters["enc_pulses_per_rev"]);

  wheel_1_.setup(cfg_.wheel1_name, cfg_.enc_pulses_per_rev);
  wheel_2_.setup(cfg_.wheel2_name, cfg_.enc_pulses_per_rev);
  wheel_3_.setup(cfg_.wheel3_name, cfg_.enc_pulses_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArceusOmniSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArceusOmniSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArceusOmniSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArceusOmniSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArceusOmniSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArceusOmniSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_POSITION, &wheel_1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_POSITION, &wheel_2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_POSITION, &wheel_3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.vel));  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArceusOmniSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn ArceusOmniSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("ArceusOmniSystemHardware"), "Activating ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("ArceusOmniSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArceusOmniSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("ArceusOmniSystemHardware"), "Deactivating ...please wait...");
  comms_.disconnect();  
  RCLCPP_INFO(rclcpp::get_logger("ArceusOmniSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArceusOmniSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  comms_.read_encoder_values(wheel_1_.enc, wheel_2_.enc, wheel_3_.enc);  

  float pos_prev = wheel_1_.pos;
  wheel_1_.pos = wheel_1_.calc_enc_angle();
  wheel_1_.vel = (wheel_1_.pos + pos_prev) / period.seconds();
  
  pos_prev = wheel_2_.pos;
  wheel_2_.pos = wheel_2_.calc_enc_angle();
  wheel_2_.vel = (wheel_2_.pos + pos_prev) / period.seconds();

  pos_prev = wheel_3_.pos;
  wheel_3_.pos = wheel_3_.calc_enc_angle();
  wheel_3_.vel = (wheel_3_.pos + pos_prev) / period.seconds(); 

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type arceus_omnidirectional_system::ArceusOmniSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  json j;
  j["motors"] = {wheel_1_.cmd, wheel_2_.cmd, wheel_3_.cmd};
  std::string data = j.dump();
  comms_.send_msg(data);

  return hardware_interface::return_type::OK;
}

}  // namespace arceus_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arceus_omnidirectional_system::ArceusOmniSystemHardware, hardware_interface::SystemInterface)