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

#ifndef VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_
#define VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_

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

#include "visibility_control.h"
#include "arduino_com.hpp"
#include "wheel.hpp"

namespace vacuum_cleaner
{
class VacuumCleanerHardware : public hardware_interface::SystemInterface
{

struct Parameters
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  int enc_counts_per_rev = 0;
  int wheel_radius_mm = 0;

  std::string motors_port = "";
  int motors_baud_rate = 0;
  int motors_timeout_ms = 0;

  std::string sensors_port = "";
  int sensors_baud_rate = 0;
  int sensors_timeout_ms = 0;

  std::string range_sensor_left_name = "";
  std::string range_sensor_front_name = "";
  std::string range_sensor_right_name = "";
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VacuumCleanerHardware);

  VACUUM_CLEANER_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  VACUUM_CLEANER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  VACUUM_CLEANER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  VACUUM_CLEANER_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  VACUUM_CLEANER_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  VACUUM_CLEANER_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  VACUUM_CLEANER_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  VACUUM_CLEANER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  VACUUM_CLEANER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  Parameters params_;
  ArduinoCom motors_;
  ArduinoCom sensors_;

  Wheel wheel_l_;
  Wheel wheel_r_;
  double test;

};

}  // namespace vacuum_cleaner

#endif  // VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_
