#ifndef ARDUINO_COM_H
#define ARDUINO_COM_H

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <json/json.h>

// commands
#define CMD_SETPOINT_LEFT        1
#define CMD_SETPOINT_RIGHT       2
#define CMD_SETPOINT_WHEELS      3
#define CMD_SETPOIN_VACUUM       4
#define CMD_ENABLE_WHEELS        5
#define CMD_ENABLE_SWEEPER       6
#define CMD_ENABLE_VACUUM        7
#define CMD_GET_ENCODER_PULSES   8
#define CMD_RESET_ENCODER_PULSES 9
#define CMD_SET_SETTINGS         10
#define CMD_GET_SETTINGS         11

class ArduinoCom 
{
public:
    ArduinoCom() = default;

    // communication
    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void disconnect();
    bool connected() const;
    
    // encoders
    void read_encoder_values(double &val_1, double &val_2);
    void reset_encoder_values();
    
    // wheels
    void enable_wheels();
    void disable_wheels();
    void set_wheel_speeds(int val_1, int val_2);

    // vacuum
    void enable_vacuum();
    void disable_vacuum();
    void set_vacuum_speed(int val_1);

    // sweeper
    void enable_sweeper();
    void disable_sweeper();

    // settings
    //void get_settings();
    //void set_settings();


private:
    LibSerial::SerialPort serial_connection_;
    int timeout_ms_;
    Json::Value settings_;
};


#endif
