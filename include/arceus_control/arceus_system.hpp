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

#ifndef ARCEUS_OMNIDIRECTIONAL_SYSTEM_HPP_
#define ARCEUS_OMNIDIRECTIONAL_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arceus_control/visibility_control.h"
#include "arceus_control/arduino_comms.hpp"
#include "arceus_control/wheel.hpp"

namespace arceus_omnidirectional_system

{
class ArceusOmniSystemHardware : public hardware_interface::SystemInterface
{

struct Config
{
    std::string wheel1_name = "";
    std::string wheel2_name = "";
    std::string wheel3_name = "";
    float loop_rate = 0.0;
    std::string device = "";
    int baud_rate = 0;
    int timeout_ms = 0;
    int enc_pulses_per_rev = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArceusOmniSystemHardware)

  ARCEUS_OMNI_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ARCEUS_OMNI_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ARCEUS_OMNI_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ARCEUS_OMNI_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ARCEUS_OMNI_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ARCEUS_OMNI_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ARCEUS_OMNI_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    
    ArduinoComms comms_;
    Config cfg_;
    Wheel wheel_1_;
    Wheel wheel_2_;
    Wheel wheel_3_;
};

}  // namespace arceus_omnidirectional_system

#endif  // ARCEUS_OMNIDIRECTIONAL_SYSTEM_HPP_