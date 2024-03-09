#ifndef VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_
#define VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "visibility_control.h"
#include "arduino_com.hpp"

namespace vacuum_cleaner
{
class VacuumCleanerHardware : public hardware_interface::SystemInterface
{

struct Communication
{
  std::string motors_port = "";
  int motors_baud_rate = 0;       // [-]
  int motors_timeout_ms = 0;      // [ms]

  std::string sensors_port = "";
  int sensors_baud_rate = 0;      // [-]
  int sensors_timeout_ms = 0;     // [ms]
};

struct Wheel
{
  std::string name = "";
  double resolution = 0;          // [-]
  double rads_per_count = 0;      // [-]
  double radius = 0;              // [mm]
  double counts = 0;              // [-]
  double command = 0;             // [rad/s]
  double setpoint = 0;            // [mm/s]
  double position = 0;            // [rad]
  double previous_positon = 0;    // [rad]
  double velocity = 0;            // [rad/s]
};

struct Vacuum
{
  std::string name = "";
  double command = 0;             // [0-255]
  double previous_command = 0;    // [0-255]
};

struct Sweeper
{
  std::string name = "";
  double command = 0;             // [1/0]
  double velocity = 0;            // [rad/s]
  double position = 0;            // [rad]
  double max_velocity = 0;        // [rad/s]
};

struct RangeSensor
{
  std::string name = "";
  double range = 0;               // [0-1023]
};

struct UserInterface
{
  std::string name = "";
  double button_circle = 0;       // [1/0]
  double button_start = 0;        // [1/0]
  double button_home = 0;         // [1/0]
  double LED_circle = 0;          // [1/0]
  double LED_start_green = 0;     // [1/0]
  double LED_start_red = 0;       // [1/0]
  double LED_home = 0;            // [1/0]
};

struct LimitSwitch
{
  std::string name = "";
  double state = 0;               // [1/0]
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
  
  Communication communication_;
  
  Wheel wheel_left_;
  Wheel wheel_right_;

  Vacuum vacuum_;

  Sweeper sweeper_left_;
  Sweeper sweeper_right_;

  RangeSensor range_sensor_left_;
  RangeSensor range_sensor_front_;
  RangeSensor range_sensor_right_;  
  
  UserInterface user_interface_;

  LimitSwitch limit_switch_left_;
  LimitSwitch limit_switch_right_;

};
}  // namespace vacuum_cleaner

#endif  // VACUUM_CLEANER__DIFFBOT_SYSTEM_HPP_
