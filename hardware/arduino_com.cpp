#include "include/arduino_com.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

std::vector<std::string> extract_values(const std::string& response) {
    std::istringstream iss(response);
    std::vector<std::string> values;
    std::string value;

    while (std::getline(iss, value, ' ')) {
        values.push_back(value);
    }

    return values;
}

// SerialCom
void SerialCom::connect(std::string serial_device, int baud_rate, int timeout_ms)
{  
  timeout_ms_ = timeout_ms;
  serial_connection_.Open(serial_device);
  serial_connection_.SetBaudRate(convert_baud_rate(baud_rate));
}

void SerialCom::disconnect()
{
  serial_connection_.Close();
}

bool SerialCom::connected() const
{
  return serial_connection_.IsOpen();
}

// MotorsCom
void MotorsCom::read_encoder_values(double &counts_left, double &counts_right)
{
  serial_connection_.FlushIOBuffers();
  serial_connection_.Write(std::to_string(CMD_GET_ENCODER_PULSES) + "\n");

  std::string response = "";
  try
  {
    serial_connection_.ReadLine(response, '\n', timeout_ms_);
  }
  catch (const LibSerial::ReadTimeout&)
  {
    std::cerr << "motor connection has timed out." << std::endl ;
  }

  std::vector<std::string> values = extract_values(response);

  counts_left = std::atoi(values[0].c_str());
  counts_right = std::atoi(values[1].c_str());
}

void MotorsCom::reset_encoder_values()
{
  std::stringstream ss;
  ss << CMD_RESET_ENCODER_PULSES << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::enable_wheels()
{
  std::stringstream ss;
  ss << CMD_ENABLE_WHEELS << " " << "true" << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::disable_wheels()
{
  std::stringstream ss;
  ss << CMD_ENABLE_WHEELS << " " << "false" << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::set_wheel_speeds(int speed_left, int speed_right)
{
  std::stringstream ss;
  ss << CMD_SETPOINT_WHEELS << " " << (speed_left) << " " << (speed_right) << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::enable_sweeper()
{
  std::stringstream ss;
  ss << CMD_ENABLE_SWEEPER << " " << "true" << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::disable_sweeper()
{
  std::stringstream ss;
  ss << CMD_ENABLE_SWEEPER << " " << "false" << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::enable_vacuum()
{
  std::stringstream ss;
  ss << CMD_ENABLE_VACUUM << " " << "true" << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::disable_vacuum()
{
  std::stringstream ss;
  ss << CMD_ENABLE_VACUUM << " " << "false" << "\n";
  serial_connection_.Write(ss.str());
}

void MotorsCom::set_vacuum_speed(int val)
{
  std::stringstream ss;
  ss << CMD_SETPOIN_VACUUM << " " << val << "\n";
  serial_connection_.Write(ss.str());
}

//void MotorsCom::get_settings()
//{
//
//}

void MotorsCom::set_settings(std::string setting, double value)
{
  std::stringstream ss;
  ss << CMD_SET_SETTINGS << " {" << (setting) << ":" << (value) << "\n";
  serial_connection_.Write(ss.str());
}

// SensorsCom
void SensorsCom::read_range_values(double &range_left, double &range_front, double &range_right)
{
  serial_connection_.FlushIOBuffers();
  serial_connection_.Write(std::to_string(CMD_GET_RANGE_VALUES) + "\n");

  std::string response = "";
  try
  {
    serial_connection_.ReadLine(response, '\n', timeout_ms_);
  }
  catch (const LibSerial::ReadTimeout&)
  {
    std::cerr << "motor connection has timed out." << std::endl ;
  }

  std::vector<std::string> values = extract_values(response);

  range_right = std::atoi(values[0].c_str());
  range_front = std::atoi(values[1].c_str());
  range_left = std::atoi(values[1].c_str());
}

//void SensorsCom::read_button_states(bool circle, bool start, bool home)
//{
//
//}
//
//void SensorsCom::set_LEDs(bool circle, bool start_green, bool start_red, bool home)
//{
//
//}
//
//void SensorsCom::read_limit_switches(bool &limit_switch_left, bool &limit_switch_right)
//{
//
//}