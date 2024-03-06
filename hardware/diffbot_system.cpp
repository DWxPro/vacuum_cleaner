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

  // setup parameters
  params_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  params_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  params_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  params_.wheel_radius_mm = std::stoi(info_.hardware_parameters["wheel_radius_mm"]);
  
  params_.motors_port = info_.hardware_parameters["motors_port"];
  params_.motors_baud_rate = std::stoi(info_.hardware_parameters["motors_baud_rate"]);
  params_.motors_timeout_ms = std::stoi(info_.hardware_parameters["motors_timeout_ms"]);

  params_.sensors_port = info_.hardware_parameters["sensors_port"];
  params_.sensors_baud_rate = std::stoi(info_.hardware_parameters["sensors_baud_rate"]);
  params_.sensors_timeout_ms = std::stoi(info_.hardware_parameters["sensors_timeout_ms"]);

  params_.range_sensor_left_name = info_.hardware_parameters["range_sensor_left_name"];
  params_.range_sensor_front_name = info_.hardware_parameters["range_sensor_front_name"];
  params_.range_sensor_right_name = info_.hardware_parameters["range_sensor_right_name"];

  wheel_l_.setup(params_.left_wheel_name, params_.enc_counts_per_rev);
  wheel_r_.setup(params_.right_wheel_name, params_.enc_counts_per_rev);

  test = 5;

  /*
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
*/
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VacuumCleanerHardware::export_state_interfaces()
{
  // export state interfaces
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.left_wheel_name, "position", &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.left_wheel_name, "velocity", &wheel_l_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.right_wheel_name, "position", &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.right_wheel_name, "velocity", &wheel_r_.vel));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.range_sensor_left_name, "range", &test));
  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.range_sensor_front_name, "range", &test));
  state_interfaces.emplace_back(hardware_interface::StateInterface(params_.range_sensor_right_name, "range", &test));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VacuumCleanerHardware::export_command_interfaces()
{
  // import command interfaces
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(params_.left_wheel_name, "velocity", &wheel_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(params_.right_wheel_name, "velocity", &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Configuring ...please wait...");

  // connect motors and sensors
  if (motors_.connected()) {motors_.disconnect();}
  if (sensors_.connected()) {sensors_.disconnect();}
  
  motors_.connect(params_.motors_port, params_.motors_baud_rate, params_.motors_timeout_ms);
  sensors_.connect(params_.sensors_port, params_.sensors_baud_rate, params_.sensors_timeout_ms);

  bool motors_connection_status = motors_.connected();
  bool sensors_connection_status = sensors_.connected();

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"Motors Connection Status: %s", motors_connection_status ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"Sensors Connection Status: %s", sensors_connection_status ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully configured!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Cleaning up ...please wait...");
  
  // disconnect motors and sensors
  if (motors_.connected()) {motors_.disconnect();}
  if (sensors_.connected()) {sensors_.disconnect();}
  
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Activating ...please wait...");
  
  // check Arduino connection before startup
  if (!motors_.connected() || !motors_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // enable wheel motors
  motors_.set_wheel_speeds(0, 0);
  motors_.enable_wheels();

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Deactivating ...please wait...");

  // disable wheel motors
  motors_.set_wheel_speeds(0, 0);
  motors_.disable_wheels();

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VacuumCleanerHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // read encoder values
  if (!motors_.connected() || !motors_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // motors
  motors_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;
 
  // sensors
  test += 1;
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"range_sensor_front: %f", test);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VacuumCleanerHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // set motor values
  if (!motors_.connected() || !motors_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"cmd wheel_l_: %f", wheel_l_.cmd);
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"motor cmd: %f", wheel_l_.cmd * params_.wheel_radius_mm);
  
  motors_.set_wheel_speeds(wheel_l_.cmd * params_.wheel_radius_mm, wheel_r_.cmd * params_.wheel_radius_mm);

  return hardware_interface::return_type::OK;
}

}  // namespace vacuum_cleaner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vacuum_cleaner::VacuumCleanerHardware, hardware_interface::SystemInterface)
