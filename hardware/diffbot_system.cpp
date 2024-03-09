#include "include/diffbot_system.hpp"

#define DEBUG(x,...)                  //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_WHEELS(x,...)           //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_VACUUM(x,...)           //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_SWEEPER(x,...)          //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_RANGE_SENSORS(x,...)    //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_LIMIT_SWITCHS(x,...)    //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_USER_INTERFACE(x,...)   //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)

namespace vacuum_cleaner
{

hardware_interface::CallbackReturn VacuumCleanerHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // check parent class initialization was successful
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // communication
  communication_.motors_port = "/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_644861E6228B85D3-if00";
  communication_.motors_baud_rate = 115200;
  communication_.motors_timeout_ms = 1000;

  communication_.sensors_port = "/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_304C61E62BB46493-if00";
  communication_.sensors_baud_rate = 115200;
  communication_.sensors_timeout_ms = 1000;

  // left wheel
  wheel_left_.name = "wheel_left_joint";
  wheel_left_.resolution = 575;
  wheel_left_.rads_per_count = ( 2 * M_PI ) / wheel_left_.resolution;
  wheel_left_.radius = 32;

  // right wheel
  wheel_right_.name = "wheel_right_joint";
  wheel_right_.resolution = 575;
  wheel_right_.rads_per_count = ( 2 * M_PI ) / wheel_right_.resolution;
  wheel_right_.radius = 32;
  
  // vacuum
  vacuum_.name = "vacuum_joint";

  // sweeper left
  sweeper_left_.name = "sweeper_left_joint";
  sweeper_left_.max_velocity = 40;

  // sweeper right
  sweeper_right_.name = "sweeper_right_joint";
  sweeper_right_.max_velocity = 40;

  // range sensors
  range_sensor_left_.name = "range_sensor_left";
  range_sensor_front_.name = "range_sensor_front";
  range_sensor_right_.name = "range_sensor_right";

  // user interface
  user_interface_.name = "user_interface";

  // limit switches
  limit_switch_left_.name = "limit_switch_left";
  limit_switch_right_.name = "limit_switch_right";

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VacuumCleanerHardware::export_state_interfaces()
{
  // export state interfaces
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_left_.name, "position", &wheel_left_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_right_.name, "position", &wheel_right_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_left_.name, "velocity", &wheel_left_.velocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_right_.name, "velocity", &wheel_right_.velocity));

  state_interfaces.emplace_back(hardware_interface::StateInterface(sweeper_left_.name, "position", &sweeper_left_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sweeper_right_.name, "position", &sweeper_right_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sweeper_left_.name, "velocity", &sweeper_left_.velocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sweeper_right_.name, "velocity", &sweeper_right_.velocity));

  state_interfaces.emplace_back(hardware_interface::StateInterface(range_sensor_left_.name, "range", &range_sensor_left_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_sensor_front_.name, "range", &range_sensor_front_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_sensor_right_.name, "range", &range_sensor_right_.range));

  state_interfaces.emplace_back(hardware_interface::StateInterface(user_interface_.name, "button_circle", &user_interface_.button_circle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(user_interface_.name, "button_start", &user_interface_.button_start));
  state_interfaces.emplace_back(hardware_interface::StateInterface(user_interface_.name, "button_home", &user_interface_.button_home));

  state_interfaces.emplace_back(hardware_interface::StateInterface(limit_switch_left_.name, "state", &limit_switch_left_.state));
  state_interfaces.emplace_back(hardware_interface::StateInterface(limit_switch_right_.name, "state", &limit_switch_right_.state));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VacuumCleanerHardware::export_command_interfaces()
{
  // import command interfaces
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_left_.name, "velocity", &wheel_left_.command));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_right_.name, "velocity", &wheel_right_.command));

//  command_interfaces.emplace_back(hardware_interface::CommandInterface(vacuum_.name, "velocity", &vacuum_.command))//;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(sweeper_left_.name, "velocity", &sweeper_left_.command));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(sweeper_right_.name, "velocity", &sweeper_right_.command));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(user_interface_.name, "LED_circle", &user_interface_.LED_circle));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(user_interface_.name, "LED_start_green", &user_interface_.LED_start_green));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(user_interface_.name, "LED_start_red", &user_interface_.LED_start_red));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(user_interface_.name, "LED_home", &user_interface_.LED_home));
  
  return command_interfaces;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Configuring ...please wait...");

  // connect motors and sensors
  if (motors_.connected()) {motors_.disconnect();}
  if (sensors_.connected()) {sensors_.disconnect();}
  
  motors_.connect(communication_.motors_port, communication_.motors_baud_rate, communication_.motors_timeout_ms);
  sensors_.connect(communication_.sensors_port, communication_.sensors_baud_rate, communication_.sensors_timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"Motors Connection Status: %s", motors_.connected() ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"),"Sensors Connection Status: %s", sensors_.connected() ? "true" : "false");
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
  
  if (!motors_.connected() || !sensors_.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "communication error");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // enable wheel motors
  motors_.set_wheel_speed_left(0);
  motors_.set_wheel_speed_right(0);

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VacuumCleanerHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Deactivating ...please wait...");

  // disable wheel motors
  motors_.set_wheel_speed_left(0);
  motors_.set_wheel_speed_right(0);

  RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VacuumCleanerHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{ 
  if (!motors_.connected() || !sensors_.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "communication error");
    return hardware_interface::return_type::ERROR;
  }

  double delta_seconds = period.seconds();

  // motors
  motors_.read_encoder_values(wheel_left_.counts, wheel_right_.counts);

  wheel_left_.position = wheel_left_.counts * wheel_left_.rads_per_count;
  wheel_right_.position = wheel_right_.counts * wheel_right_.rads_per_count;

  wheel_left_.velocity = ( wheel_left_.position - wheel_left_.previous_positon ) / delta_seconds;   
  wheel_right_.velocity = ( wheel_right_.position - wheel_right_.previous_positon ) / delta_seconds;   
  
  wheel_left_.previous_positon = wheel_left_.position;
  wheel_right_.previous_positon = wheel_right_.position;

  sweeper_left_.position += sweeper_left_.velocity * delta_seconds;
  sweeper_right_.position += sweeper_right_.velocity * delta_seconds;

  // sensors
  sensors_.read_range_values(range_sensor_left_.range, range_sensor_front_.range, range_sensor_right_.range);

//  sensors_.read_button_states(user_interface_.button_circle, user_interface_.button_start, user_interface_.button_home);

//  sensors_.read_limit_switches(limit_switch_left_.state, limit_switch_right_.state);

  DEBUG_WHEELS("encoder left: %f encoder right: %f", wheel_left_.counts, wheel_right_.counts);
  DEBUG_RANGE_SENSORS("range left: %f range front: %f range right: %f", range_sensor_left_.range, range_sensor_front_.range, range_sensor_right_.range);
  DEBUG_USER_INTERFACE("button circle: %f button start: %f button home: %f", user_interface_.button_circle, user_interface_.button_start, user_interface_.button_home);
  DEBUG_LIMIT_SWITCHS("limit switch left: %f limit siwtch right: %f" , limit_switch_left.state, limit_switch_right.state);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VacuumCleanerHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motors_.connected() || !sensors_.connected()){
    return hardware_interface::return_type::ERROR;
    RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "communication error");
  }

  // left wheel
  if(wheel_left_.command != 0){
    motors_.enable_wheel_left();
    wheel_left_.setpoint = wheel_left_.command * wheel_left_.radius;
    motors_.set_wheel_speed_left(wheel_left_.setpoint);
  }
  else{
    motors_.disable_wheel_left();
  }

  // right wheel
  if(wheel_right_.command != 0){
    motors_.enable_wheel_right();
    wheel_right_.setpoint = wheel_right_.command * wheel_right_.radius;
    motors_.set_wheel_speed_right(wheel_right_.setpoint);
  }
  else{
    motors_.disable_wheel_left();
  }

  // vacuum
//  motors_.set_vacuum_speed(vacuum_.command);  
//  if (vacuum_.previous_command == 0 && vacuum_.command > 0) {motors_.enable_vacuum();}
//  else if (vacuum_.command == 0) {motors_.disable_vacuum();}
  
  // sweeper left
  if (sweeper_left_.command == sweeper_left_.max_velocity){
    motors_.enable_sweeper_left();
    sweeper_left_.velocity = sweeper_left_.max_velocity;
  }
  else{
    motors_.disable_sweeper_left();
    sweeper_left_.velocity = 0;
  }

  // sweeper right
  if (sweeper_right_.command == sweeper_right_.max_velocity){
    motors_.enable_sweeper_right();
    sweeper_right_.velocity = sweeper_right_.max_velocity;
  }
  else{
    motors_.disable_sweeper_right();
    sweeper_right_.velocity = 0;
  }

//  // sensors
//  sensors_.set_LEDs(user_interface_.LED_circle, user_interface_.LED_start_green, user_interface_.LED_start_red, user_interface_.button_home);

  DEBUG_WHEELS("command left: %f comand right: %f", wheel_left_.command, wheel_right_.command);
  DEBUG_WHEELS("setpoint left: %f setpoint right: %f", wheel_left_.setpoint, wheel_right_.setpoint);
  DEBUG_VACUUM("command: %f ", vacuum_.command);
  DEBUG_SWEEPER("sweeper left: %f sweeper right: %f", sweeper_left_.command, sweeper_right_.command);
  return hardware_interface::return_type::OK;
}
}  // namespace vacuum_cleaner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vacuum_cleaner::VacuumCleanerHardware, hardware_interface::SystemInterface)

// vacuum_cleaner::VacuumCleanerHardware must match the class in the XML file