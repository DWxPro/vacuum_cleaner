#ifndef ARDUINO_COM_H
#define ARDUINO_COM_H

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <json/json.h>

// motor commands
#define CMD_SETPOINT_LEFT             1   // [mm/s]
#define CMD_SETPOINT_RIGHT            2   // [mm/s]
#define CMD_SETPOINT_WHEELS           3   // [left,right]
#define CMD_SETPOIN_VACUUM            4   // [0-255]
#define CMD_ENABLE_WHEELS             5   // [true/false]
#define CMD_ENABLE_SWEEPER            6   // [true/false]
#define CMD_ENABLE_VACUUM             7   // [true/false]
#define CMD_GET_ENCODER_PULSES        8   // [-]
#define CMD_RESET_ENCODER_PULSES      9   // [-]
#define CMD_SET_SETTINGS              10  // [-]
#define CMD_GET_SETTINGS              11  // [-]

// sensor commands
#define CMD_GET_RANGE_VALUES          1   // [right,front,left]
#define CMD_GET_BUTTON_STATES         2   // [circle,start,home]
#define CMD_GET_LIMIT_SWITCH_STATES   3   // [right,left]
#define CMD_SET_LEDS                  4   // [circle,start-grün,start-rot,home]

class ArduinoCom 
{
public:
    ArduinoCom() = default;

    // communication
    void connect(std::string serial_device, int baud_rate, int timeout_ms);
    void disconnect();
    bool connected() const;
    
    // encoders
    void read_encoder_values(double &counts_left, double &counts_right);
    void reset_encoder_values();
    
    // wheels
    void enable_wheels();
    void disable_wheels();
    void set_wheel_speeds(int speed_left, int speed_right);

    // vacuum
    void enable_vacuum();
    void disable_vacuum();
    void set_vacuum_speed(int speed);

    // sweeper
    void enable_sweeper();
    void disable_sweeper();

    // settings
    //void get_settings();
    //void set_settings();

    // range sensors
    void read_range_values(double &range_left, double &range_front, double &range_right);

    // buttons
    void read_button_states(bool circle, bool start, bool home);
    void set_LEDs(bool circle, bool start_green, bool start_red, bool home);

    // limit switches
    void read_limit_switches(bool &limit_switch_left, bool &limit_switch_right);

private:
    LibSerial::SerialPort serial_connection_;
    int timeout_ms_;
    Json::Value settings_;
};


#endif
