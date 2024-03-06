#include "include/diffbot_system.hpp"

#define DEBUG_RANGE(x,...)    //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)
#define DEBUG_WHEELS(x,...)   //RCLCPP_INFO(rclcpp::get_logger("##### DEBUG #####"), x, ##__VA_ARGS__)

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
  communication_.motors_port = info_.hardware_parameters["motors_port"];
  communication_.motors_baud_rate = std::stoi(info_.hardware_parameters["motors_baud_rate"]);
  communication_.motors_timeout_ms = std::stoi(info_.hardware_parameters["motors_timeout_ms"]);

  communication_.sensors_port = info_.hardware_parameters["sensors_port"];
  communication_.sensors_baud_rate = std::stoi(info_.hardware_parameters["sensors_baud_rate"]);
  communication_.sensors_timeout_ms = std::stoi(info_.hardware_parameters["sensors_timeout_ms"]);

  // left wheel
  wheel_left_.name = info_.hardware_parameters["left_wheel_name"];
  wheel_left_.rads_per_count = ( 2 * M_PI ) / std::stoi(info_.hardware_parameters["left_encoder_resolution"]);
  wheel_left_.radius = std::stoi(info_.hardware_parameters["left_wheel_radius"]);

  // right wheel
  wheel_right_.name = info_.hardware_parameters["right_wheel_name"];
  wheel_right_.rads_per_count = ( 2 * M_PI ) / std::stoi(info_.hardware_parameters["right_encoder_resolution"]);
  wheel_right_.radius = std::stoi(info_.hardware_parameters["right_wheel_radius"]);
  
  // range sensors
  range_sensor_left_.name = info_.hardware_parameters["range_sensor_left_name"];
  range_sensor_front_.name = info_.hardware_parameters["range_sensor_front_name"];
  range_sensor_right_.name = info_.hardware_parameters["range_sensor_right_name"];

  // user interface
  home_button_.name = info_.hardware_parameters["home_button_name"];
  home_button_LED_.name = info_.hardware_parameters["home_button_LED_name"];

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VacuumCleanerHardware::export_state_interfaces()
{
  // export state interfaces
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_left_.name, "position", &wheel_left_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_left_.name, "velocity", &wheel_left_.velocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_right_.name, "position", &wheel_right_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_right_.name, "velocity", &wheel_right_.velocity));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_sensor_left_.name, "range", &range_sensor_left_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_sensor_front_.name, "range", &range_sensor_front_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_sensor_right_.name, "range", &range_sensor_right_.range));

  state_interfaces.emplace_back(hardware_interface::StateInterface("user_interface", "home_button", &home_button_.state));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VacuumCleanerHardware::export_command_interfaces()
{
  // import command interfaces
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_left_.name, "velocity", &wheel_left_.command));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_right_.name, "velocity", &wheel_right_.command));

  command_interfaces.emplace_back(hardware_interface::CommandInterface("user_interface", "home_button_LED", &home_button_LED_.state));

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

  bool motors_connection_status = motors_.connected();
  bool sensors_connection_status = sensors_.connected();

  // print connection status
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
  if (!motors_.connected() || !sensors_.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "communication error");
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
    RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "communication error");
    return hardware_interface::return_type::ERROR;
  }

  // motors
  motors_.read_encoder_values(wheel_left_.counts, wheel_right_.counts);

  wheel_left_.position = wheel_left_.counts * wheel_left_.rads_per_count;
  wheel_left_.position = wheel_left_.counts * wheel_left_.rads_per_count;

  double delta_seconds = period.seconds();
  wheel_left_.velocity = ( wheel_left_.position - wheel_left_.previous_positon ) / delta_seconds;   
  wheel_right_.velocity = ( wheel_right_.position - wheel_right_.previous_positon ) / delta_seconds;   
  
  wheel_left_.previous_positon = wheel_left_.position;
  wheel_right_.previous_positon = wheel_right_.position;

  // sensors
  sensors_.read_range_values(range_sensor_left_.range, range_sensor_front_.range, range_sensor_right_.range);
  
  DEBUG_RANGE("range left: %f range front: %f range right: %f", range_sensor_left_.range, range_sensor_front_.range, range_sensor_right_.range);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VacuumCleanerHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // set motor values
  if (!motors_.connected() || !motors_.connected())
  {
    return hardware_interface::return_type::ERROR;
    RCLCPP_INFO(rclcpp::get_logger("vacuumCleanerHardware"), "communication error");
  }
  
  motors_.set_wheel_speeds(wheel_left_.command * wheel_left_.radius, wheel_right_.command * wheel_right_.radius);

  DEBUG_WHEELS("command left: %f comand right: %f", wheel_left_.command, wheel_right_.command);
  DEBUG_WHEELS("speed left: %f speed right: %f", wheel_left_.command * wheel_left_.radius, wheel_right_.command * wheel_right_.radius);
  return hardware_interface::return_type::OK;
}

}  // namespace vacuum_cleaner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vacuum_cleaner::VacuumCleanerHardware, hardware_interface::SystemInterface)

// vacuum_cleaner::VacuumCleanerHardware must match the class in the XML file