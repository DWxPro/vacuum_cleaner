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

#include "include/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vacuum_cleaner
{

hardware_interface::CallbackReturn VacuumCleanerHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // check parent class initialization was successful
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // setup 
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  
  cfg_.serial_device = info_.hardware_parameters["serial_device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


  // check URDF configurations
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("vacuumCleanerHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("vacuumCleanerHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("vacuumCleanerHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("vacuumCleanerHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("vacuumCleanerHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VacuumCleanerHardware::export_state_interfaces()
{
  // generate state interfaces
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VacuumCleanerHardware::export_command_interfaces()
{
  // generate command interfaces
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Configuring ...please wait...");

  // connect Arduino
  if (arduino_.connected())
  {
    arduino_.disconnect();
  }

  arduino_.connect(cfg_.serial_device, cfg_.baud_rate, cfg_.timeout_ms);

  bool status = arduino_.connected();

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"Arduino Connection Status: %s",status ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully configured!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Cleaning up ...please wait...");
  
  // disconnect Arduino
  if (arduino_.connected())
  {
    arduino_.disconnect();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Activating ...please wait...");
  
  // check Arduino connection before startup
  if (!arduino_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // enable wheel motors
  arduino_.set_wheel_speeds(0, 0);
  arduino_.enable_wheels();

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Deactivating ...please wait...");

  // disable wheel motors
  arduino_.set_wheel_speeds(0, 0);
  arduino_.disable_wheels();

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VacuumCleanerHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // read encoder values
  if (!arduino_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  arduino_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  // calculate position and velocity
  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;
 
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VacuumCleanerHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // set motor values
  if (!arduino_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  arduino_.set_wheel_speeds(wheel_l_.cmd, wheel_r_.cmd);

  // RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"Encoder left: %f Encoder right %f", wheel_l_.cmd, wheel_r_.cmd);
  // use ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

  return hardware_interface::return_type::OK;
}

}  // namespace vacuum_cleaner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vacuum_cleaner::VacuumCleanerHardware, hardware_interface::SystemInterface)
