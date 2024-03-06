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

void ArduinoCom::connect(std::string serial_device, int baud_rate, int timeout_ms)
{  
  timeout_ms_ = timeout_ms;
  serial_connection_.Open(serial_device);
  serial_connection_.SetBaudRate(convert_baud_rate(baud_rate));
}

void ArduinoCom::disconnect()
{
  serial_connection_.Close();
}

bool ArduinoCom::connected() const
{
  return serial_connection_.IsOpen();
}

void ArduinoCom::read_encoder_values(double &counts_left, double &counts_right)
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

  // TODO: change seperation

  std::string delimiter = " ";
  size_t del_pos = response.find(delimiter);
  std::string pulses_left = response.substr(0, del_pos);
  std::string pulses_right = response.substr(del_pos + delimiter.length());

  counts_left = std::atoi(pulses_left.c_str());
  counts_right = std::atoi(pulses_right.c_str());
}

void ArduinoCom::reset_encoder_values()
{
  std::stringstream ss;
  ss << CMD_RESET_ENCODER_PULSES << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::enable_wheels()
{
  std::stringstream ss;
  ss << CMD_ENABLE_WHEELS << " " << "true" << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::disable_wheels()
{
  std::stringstream ss;
  ss << CMD_ENABLE_WHEELS << " " << "false" << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::set_wheel_speeds(int speed_left, int speed_right)
{
  std::stringstream ss;
  ss << CMD_SETPOINT_WHEELS << " " << (speed_left) << " " << (speed_right) << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::enable_sweeper()
{
  std::stringstream ss;
  ss << CMD_ENABLE_SWEEPER << " " << "true" << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::disable_sweeper()
{
  std::stringstream ss;
  ss << CMD_ENABLE_SWEEPER << " " << "false" << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::enable_vacuum()
{
  std::stringstream ss;
  ss << CMD_ENABLE_VACUUM << " " << "true" << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::disable_vacuum()
{
  std::stringstream ss;
  ss << CMD_ENABLE_VACUUM << " " << "false" << "\n";
  serial_connection_.Write(ss.str());
}

void ArduinoCom::set_vacuum_speed(int val)
{
  std::stringstream ss;
  ss << CMD_SETPOIN_VACUUM << " " << val << "\n";
  serial_connection_.Write(ss.str());
}

