#ifndef VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_
#define VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_

//#include <memory>
//#include <string>
//#include <vector>

//#include "hardware_interface/handle.hpp"
//#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
//#include "hardware_interface/types/hardware_interface_return_values.hpp"
//#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/clock.hpp"
//#include "rclcpp/duration.hpp"
//#include "rclcpp/macros.hpp"
//#include "rclcpp/time.hpp"
//#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
//#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"
#include "arduino_com.hpp"

namespace vacuum_cleaner
{
class VacuumCleanerHardware : public hardware_interface::SystemInterface
{

struct CommunicationParameters
{
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

struct WheelParameters
{
  std::string name = "";
  double counts = 0;            // [-]
  double command = 0;           // [rad/s]
  double position = 0;          // [rad]
  double previous_positon = 0;  // [rad]
  double velocity = 0;          // [rad/s]
  double rads_per_count = 0;    // [-]
  double radius = 0;            // [mm]
};

struct RangeSensorParameters
{
  std::string name = "";
  double range = 0;             // [0-1023]
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

  MotorsCom motors_;
  SensorsCom sensors_;
  
  CommunicationParameters communication_;
  
  WheelParameters wheel_left_;
  WheelParameters wheel_right_;

  RangeSensorParameters range_sensor_left_;
  RangeSensorParameters range_sensor_front_;
  RangeSensorParameters range_sensor_right_;  
  
};

}  // namespace vacuum_cleaner

#endif  // VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_
