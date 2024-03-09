#ifndef ARDUINO_COM_H
#define ARDUINO_COM_H

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <json/json.h>

// motor commands
#define CMD_SETPOINT_LEFT             1   // [mm/s]
#define CMD_SETPOINT_RIGHT            2   // [mm/s]
#define CMD_SETPOINT_VACUUM           3   // [0-255]
#define CMD_ENABLE_WHEEL_LEFT         4   // [true/false]
#define CMD_ENABLE_WHEEL_RIGHT        5   // [true/false]
#define CMD_ENABLE_SWEEPER_LEFT       6   // [true/false]
#define CMD_ENABLE_SWEEPER_RIGHT      7   // [true/false]
#define CMD_ENABLE_VACUUM             8   // [true/false]
#define CMD_GET_ENCODER_PULSES        9   // [-]
#define CMD_RESET_ENCODER_PULSES      10  // [-]
#define CMD_SET_SETTINGS              11  // [-]
#define CMD_GET_SETTINGS              12  // [-]

// sensor commands
#define CMD_GET_RANGE_VALUES          1   // [right,front,left]
#define CMD_GET_BUTTON_STATES         2   // [circle,start,home]
#define CMD_GET_LIMIT_SWITCH_STATES   3   // [right,left]
#define CMD_SET_LEDS                  4   // [circle,start-grün,start-rot,home]

class SerialCom 
{
public:
    SerialCom() = default;

    // communication
    void connect(std::string serial_device, int baud_rate, int timeout_ms);
    void disconnect();
    bool connected() const;

    LibSerial::SerialPort serial_connection_;
    int timeout_ms_;
};

class MotorsCom : public SerialCom
{
public:
    // encoders
    void read_encoder_values(double &counts_left, double &counts_right);
    void reset_encoder_values();
    
    // left wheel
    void enable_wheel_left();
    void disable_wheel_left();
    void set_wheel_speed_left(int speed_left);

    // right wheel
    void enable_wheel_right();
    void disable_wheel_right();
    void set_wheel_speed_right(int speed_right);

    // vacuum
    void enable_vacuum();
    void disable_vacuum();
    void set_vacuum_speed(int speed);

    // sweeper
    void enable_sweeper_left();
    void disable_sweeper_left();
    void enable_sweeper_right();
    void disable_sweeper_right();

    // settings
    void get_settings();
    void set_settings(std::string setting, double value);
};

class SensorsCom : public SerialCom
{
public:
    // range sensors
    void read_range_values(double &range_left, double &range_front, double &range_right);

    // buttons
    void read_button_states(bool circle, bool start, bool home);
    void set_LEDs(bool circle, bool start_green, bool start_red, bool home);

    // limit switches
    void read_limit_switches(bool &limit_switch_left, bool &limit_switch_right);
};

#endif
