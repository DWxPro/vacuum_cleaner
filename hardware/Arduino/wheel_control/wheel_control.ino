#include <Arduino.h>
#include <Scheduler.h>
#include <ArduinoJson.h>
#include "pid.h"

// debug
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)

// temp values
int i_temp;
double d_temp;
String S_temp;

// commands
#define CMD_SETPOINT_LEFT        1   // [mm/s]
#define CMD_SETPOINT_RIGHT       2   // [mm/s]
#define CMD_SETPOINT_WHEELS      3   // [left,right]
#define CMD_SETPOIN_VACCUM       4   // [0-255]
#define CMD_ENABLE_WHEELS        5   // [true/false]
#define CMD_ENABLE_SWEEPER       6   // [true/false]
#define CMD_ENABLE_VACCUM        7   // [true/false]
#define CMD_GET_ENCODER_PULSES   8   // [-]
#define CMD_RESET_ENCODER_PULSES 9   // [-]
#define CMD_SET_SETTINGS         10  // [-]
#define CMD_GET_SETTINGS         11  // [-]

String message;
int command;
String value;

// pin settings

#define PIN_WHEEL_RIGHT_ENABLE 2
#define PIN_WHEEL_RIGHT_FORWARD 4
#define PIN_WHEEL_RIGHT_BACKWARD 3
#define PIN_WHEEL_RIGHT_ENCODER 28

#define PIN_WHEEL_LEFT_ENABLE 7
#define PIN_WHEEL_LEFT_FORWARD 5
#define PIN_WHEEL_LEFT_BACKWARD 6
#define PIN_WHEEL_LEFT_ENCODER 27

#define PIN_SWEEPER_LEFT_ENABLE 12
#define PIN_SWEEPER_RIGHT_ENABLE 13

#define PIN_VACCUM_ENABLE 1
#define PIN_VACCUM_PWM 10

// settings JSON
const size_t jsonCapacity = JSON_OBJECT_SIZE(8);
DynamicJsonDocument settings_JSON(jsonCapacity);
DynamicJsonDocument new_settings_JSON(jsonCapacity);

// control parameters
double kp_left = 0.8;               // [-]
double ki_left = 0.03;               // [ms]
double kd_left = 0.2;               // [mm/ms²]

double kp_right = 0.8;              // [-] 
double ki_right = 0.03;              // [ms]
double kd_right = 0.2;              // [mm/ms²]

bool enable_controller = false;

double setpoint_left = 0.0;         // [mm/s]
double setpoint_right = 0.0;        // [mm/s]
double speed_left = 0.0;            // [mm/s]
double speed_right = 0.0;           // [mm/s]
double control_value_left = 0.0;    // [%]
double control_value_right = 0.0;   // [%]

double max_PWM = 255;               // [%]
double min_PWM = 0;                 // [%]

#define CYCLE_TIME 30               // [ms]
#define START_SERIAL_DELAY 4        // [ms]

unsigned long time_last_cycle = 0;  // [ms]
unsigned long time_now = 0;         // [ms]
int time_delta = 0;                 // [ms]

PID PID_left = PID(CYCLE_TIME, max_PWM, min_PWM, kp_left, kd_left, ki_left);
PID PID_right = PID(CYCLE_TIME, max_PWM, min_PWM, kp_right, kd_right, ki_right);

// encoder
double encoder_resolution = 575;  // [-]
double wheel_diameter = 64;       // [mm]

#define PI 3.141592653589793      // [-]

double lengthPerPulse = PI * wheel_diameter  / encoder_resolution;

unsigned long encoder_time_now_left = 0;          // [µm]
unsigned long encoder_time_last_left = 0;         // [µm]
unsigned long encoder_time_delta_left = 3000000;  // [µm] default value simulates very slow start motion.
long encoder_pulses_left = 0;                     // [-]

unsigned long encoder_time_now_right = 0;         // [µm]
unsigned long encoder_time_last_right = 0;        // [µm]
unsigned long encoder_time_delta_right = 3000000; // [µm]
long encoder_pulses_right = 0;                    // [-]

void calculateEncoderTimeLeft(){
  encoder_time_now_left = micros();
  encoder_time_delta_left = encoder_time_now_left - encoder_time_last_left;
  encoder_time_last_left = encoder_time_now_left;

  if(setpoint_left > 0){
    encoder_pulses_left += 1;
  }
  else if(setpoint_left < 0){
    encoder_pulses_left -= 1;
  }
}

void calculateEncoderTimeRight(){
  encoder_time_now_right = micros();
  encoder_time_delta_right = encoder_time_now_right - encoder_time_last_right;
  encoder_time_last_right = encoder_time_now_right;
  
  if(setpoint_right > 0){
    encoder_pulses_right += 1;
  }
  else if(setpoint_right < 0){
    encoder_pulses_right -= 1;
  }
}

// sweeper
bool enable_sweeper = false;

// vaccum
bool enable_vaccum = false;
int vaccum_speed = 0;

void setup() {
  Serial.begin(115200);

  // pin settings
  pinMode(PIN_WHEEL_LEFT_ENABLE, OUTPUT);
  pinMode(PIN_WHEEL_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_WHEEL_LEFT_BACKWARD, OUTPUT);
  pinMode(PIN_WHEEL_LEFT_ENABLE, OUTPUT);
  pinMode(PIN_WHEEL_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_WHEEL_LEFT_BACKWARD, OUTPUT);
  pinMode(PIN_SWEEPER_LEFT_ENABLE, OUTPUT);
  pinMode(PIN_SWEEPER_RIGHT_ENABLE, OUTPUT);
  pinMode(PIN_VACCUM_ENABLE, OUTPUT);
  pinMode(PIN_VACCUM_PWM, OUTPUT);

  digitalWrite(PIN_WHEEL_LEFT_ENABLE, LOW);
  digitalWrite(PIN_WHEEL_RIGHT_ENABLE, LOW);
  digitalWrite(PIN_VACCUM_ENABLE, LOW);

  // encoder
  attachInterrupt(PIN_WHEEL_LEFT_ENCODER, calculateEncoderTimeLeft, RISING);
  attachInterrupt(PIN_WHEEL_RIGHT_ENCODER, calculateEncoderTimeRight, RISING);
  interrupts();

}

void loop(){

  // run controller
  if (enable_controller){

    time_now = millis();
    time_delta = time_now - time_last_cycle;

    if (time_delta >= CYCLE_TIME){

      speed_left = (lengthPerPulse / encoder_time_delta_left) * 1000000;
      speed_right = (lengthPerPulse / encoder_time_delta_right) * 1000000;

      control_value_left = PID_left.calculate(abs(setpoint_left), speed_left);
      control_value_right = PID_right.calculate(abs(setpoint_right), speed_right);
        
      if (setpoint_left >= 0){  
        analogWrite(PIN_WHEEL_LEFT_FORWARD, control_value_left);
        analogWrite(PIN_WHEEL_LEFT_BACKWARD, 0);
      }
      else{
        analogWrite(PIN_WHEEL_LEFT_FORWARD, 0);
        analogWrite(PIN_WHEEL_LEFT_BACKWARD, control_value_left);
      }

      if (setpoint_right >= 0){  
        analogWrite(PIN_WHEEL_RIGHT_FORWARD, control_value_right);
        analogWrite(PIN_WHEEL_RIGHT_BACKWARD, 0);
      }
      else{
        analogWrite(PIN_WHEEL_RIGHT_FORWARD, 0);
        analogWrite(PIN_WHEEL_RIGHT_BACKWARD, control_value_right);
      }
    
/* use to setup PID */
      debug(" set_r: ");
      debug(setpoint_right);
      debug(" set_l: ");    
      debug(setpoint_left);
      
      //debug(" cmd_r: ");
      //debug(control_value_right);
      //debug(" cmd_l: ");
      //debug(control_value_left);
    
      debug(" spe_r: ");
      debug(speed_right);
      debug(" spe_l: ");
      debugln(speed_left);

      time_last_cycle = time_now;
      time_delta = 0;
    }
  }

  else{
    time_delta = 0;
  }

  // process commands 
  if (Serial.available() > 0)
  {
    message = Serial.readStringUntil('\n');
    command = message.substring(0, message.indexOf(' ')).toInt();
    value = message.substring(message.indexOf(' ')+1);

    switch (command)
    {

      case CMD_GET_ENCODER_PULSES:{
        debug("(" + String(CMD_GET_ENCODER_PULSES) + ") ");
        Serial.println(String(encoder_pulses_left) + " " + String(encoder_pulses_right));
        break;
      }

      case CMD_SETPOINT_WHEELS:{
        setpoint_left = value.substring(0, value.indexOf(' ')).toDouble();
        setpoint_right = value.substring(value.indexOf(' ')+1).toDouble();
        
        debugln("(" + String(CMD_SETPOINT_WHEELS) + ")" + " setpoint left: "  + String(setpoint_left,3) 
                                                        + " setpoint right: " + String(setpoint_right,3));
        break;
      }

      case CMD_SETPOIN_VACCUM:{
        vaccum_speed = value.toInt();
        analogWrite(PIN_VACCUM_PWM, vaccum_speed);
          
        debugln("(" + String(CMD_SETPOIN_VACCUM) + ")" + " vaccum speed: " + vaccum_speed);
        break;
      }

      case CMD_ENABLE_WHEELS:{
        enable_controller = (value == "true");
        if (enable_controller){
          digitalWrite(PIN_WHEEL_LEFT_ENABLE, HIGH);
          digitalWrite(PIN_WHEEL_RIGHT_ENABLE, HIGH);
        }
        else{
          digitalWrite(PIN_WHEEL_LEFT_ENABLE, LOW);
          digitalWrite(PIN_WHEEL_RIGHT_ENABLE, LOW);
        }

        debugln("(" + String(CMD_ENABLE_WHEELS) + ")" + " wheels: " + (String((enable_controller) ? "true" : "false")));
        break;
      }

      case CMD_ENABLE_SWEEPER:{
        enable_sweeper = (value == "true");
        if (enable_sweeper){
          digitalWrite(PIN_SWEEPER_LEFT_ENABLE, HIGH);
          digitalWrite(PIN_SWEEPER_RIGHT_ENABLE, HIGH);
        }
        else{
          digitalWrite(PIN_SWEEPER_LEFT_ENABLE, LOW);
          digitalWrite(PIN_SWEEPER_RIGHT_ENABLE, LOW);
        }
        
        debugln("(" + String(CMD_ENABLE_SWEEPER) + ")" + " sweeper: " + (String((enable_sweeper) ? "true" : "false")));
        break;
      }

      case CMD_ENABLE_VACCUM:{
        enable_vaccum = (value == "true");
        if (enable_vaccum){
          digitalWrite(PIN_VACCUM_ENABLE, HIGH);
        }
        else{
          digitalWrite(PIN_VACCUM_ENABLE, LOW);
        }
        
        debugln("(" + String(CMD_ENABLE_VACCUM) + ")" + " vaccum: " + (String((enable_vaccum) ? "true" : "false")));
        break;
      }

      case CMD_RESET_ENCODER_PULSES:{
        encoder_pulses_left = 0;
        encoder_pulses_right = 0;
        debugln("(" + String(CMD_RESET_ENCODER_PULSES) + ")" + " encoder pulses reseted");
        break;
      }

      case CMD_SETPOINT_LEFT:{
        setpoint_left = value.toFloat();

        debugln("(" + String(CMD_SETPOINT_LEFT) + ")" + " setpoint left: " + String(setpoint_left,1));
        break;
      }

      case CMD_SETPOINT_RIGHT:{
        setpoint_right = value.toFloat();

        debugln("(" + String(CMD_SETPOINT_RIGHT) + ")" + " setpoint right: " + String(setpoint_right,1));
        break;
      }

      case CMD_SET_SETTINGS:{
        deserializeJson(new_settings_JSON, value);

        if (new_settings_JSON.containsKey("kp_left")) {kp_left = new_settings_JSON["kp_left"];}
        if (new_settings_JSON.containsKey("ki_left")) {ki_left = new_settings_JSON["ki_left"];}
        if (new_settings_JSON.containsKey("kd_left")) {kd_left = new_settings_JSON["kd_left"];}
        if (new_settings_JSON.containsKey("kp_right")) {kp_right = new_settings_JSON["kp_right"];}
        if (new_settings_JSON.containsKey("ki_right")) {ki_right = new_settings_JSON["ki_right"];}
        if (new_settings_JSON.containsKey("kd_right")) {kd_right = new_settings_JSON["kd_right"];}
        if (new_settings_JSON.containsKey("encoder_resolution")) {encoder_resolution = new_settings_JSON["encoder_resolution"];}
        if (new_settings_JSON.containsKey("wheel_diameter")) {wheel_diameter = new_settings_JSON["wheel_diameter"];}

        PID_left.setPID(kp_left, kd_left, ki_left);
        PID_right.setPID(kp_right, kd_right, ki_right);

        //PID PID_left = PID(CYCLE_TIME, max_PWM, min_PWM, kp_left, kd_left, ki_left);
        //PID PID_right = PID(CYCLE_TIME, max_PWM, min_PWM, kp_right, kd_right, ki_right);

        lengthPerPulse = 2 * (wheel_diameter / 2) * PI / encoder_resolution;

        debugln("(" + String(CMD_SET_SETTINGS) + ") ");
        debugln("kp left: " + String(kp_left,3));
        debugln("ki left: " + String(ki_left,3));
        debugln("kd left: " + String(kd_left,3));
        debugln("kp right: " + String(kp_right,3));
        debugln("ki right: " + String(ki_right,3));
        debugln("kd right: " + String(kd_right,3));
        debugln("encoder resolution: " + String(encoder_resolution,3));
        debugln("wheel diameter: " + String(wheel_diameter,3));
        break;
      }

      case CMD_GET_SETTINGS:{
        settings_JSON["kp_left"] = kp_left;
        settings_JSON["ki_left"] = ki_left;
        settings_JSON["kd_left"] = kd_left;
        settings_JSON["kp_right"] = kp_right;
        settings_JSON["ki_right"] = ki_right;
        settings_JSON["kd_right"] = kd_right;
        settings_JSON["encoder_resolution"] = encoder_resolution;
        settings_JSON["wheel_diameter"] = wheel_diameter;

        String settings_msg;
        serializeJson(settings_JSON, settings_msg);
        debug("(" + String(CMD_GET_SETTINGS) + ") ");
        Serial.println(settings_msg);
        break;
      }

      case 0:{
        Serial.println("incorrect command");
        break;
      }

      default:{
        Serial.println("incorrect command");
        break;
      }

    }
  }
}


