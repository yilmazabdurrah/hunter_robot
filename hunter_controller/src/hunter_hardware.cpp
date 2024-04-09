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

#include "hunter_hardware/hunter_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hunter_hardware
{
hardware_interface::CallbackReturn HunterHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HunterHardware::export_state_interfaces()
{
std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
  // return 1;
}

std::vector<hardware_interface::CommandInterface> HunterHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  return command_interfaces;
}

hardware_interface::CallbackReturn HunterHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HunterHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HunterHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HunterHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HunterHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hunter_hardware ::HunterHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}

}  // namespace hunter_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hunter_hardware::HunterHardware, hardware_interface::SystemInterface)
