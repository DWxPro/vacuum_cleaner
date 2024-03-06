#include <Arduino.h>
#include <Scheduler.h>

// debug
#define debugSensor(x) //Serial.print(x)
#define debugSensorln(x) //Serial.println(x)
#define debugCom(x) //Serial.print(x)
#define debugComln(x) //Serial.println(x)

// temp values
int i_temp;
double d_temp;
String S_temp;

// commands
#define CMD_GET_RANGE_VALUES          1   // [right,front,left]
#define CMD_GET_BUTTON_STATES         2   // [circle,start,home]
#define CMD_GET_LIMIT_SWITCH_STATES   3   // [right,left]
#define CMD_SET_LEDS                  4   // [circle,start-grün,start-rot,home]

String message;
int command;
String value;

// control panel
#define PIN_BUTTON_CIRCLE 2
#define PIN_BUTTON_START 3
#define PIN_BUTTON_HOME 4
#define PIN_LED_START_GREEN 5
#define PIN_LED_START_RED 6
#define PIN_LED_CIRCLE 7
#define PIN_LED_HOME 8

bool button_state_circle = false;
bool button_state_start = false;
bool button_state_home = false;

bool LED_state_circle = false;
bool LED_state_start_green = false;
bool LED_state_start_red = false;
bool LED_state_home = false;

// range sensor
#define PIN_SIGNAL 22
#define PIN_SENSOR_RIGHT 26
#define PIN_SENSOR_FRONT 27
#define PIN_SENSOR_LEFT 28

int range_right = 0;
int range_front = 0;
int range_left = 0;

#define CYCLE_TIME 30               // [ms]

unsigned long time_last_cycle = 0;  // [ms]
unsigned long time_now = 0;         // [ms]
int time_delta = 0;                 // [ms]

// limit switchs
#define PIN_LIMIT_SWITCH_RIGHT 14
#define PIN_LIMIT_SWITCH_LEFT 15

bool limit_switch_state_right = false;
bool limit_switch_state_left = false;

void readRangeSensors(int &range_right, int &range_front, int &range_left){

  time_now = millis();
  time_delta = time_now - time_last_cycle;

  if (time_delta >= CYCLE_TIME){
    digitalWrite(PIN_SIGNAL, LOW);

    delay(1);  

    range_right = analogRead(PIN_SENSOR_RIGHT);
    range_left = analogRead(PIN_SENSOR_LEFT);
    range_front = analogRead(PIN_SENSOR_FRONT);

    debugSensor("left ");
    debugSensor(range_left);
    debugSensor(" front ");
    debugSensor(range_front);
    debugSensor(" right ");
    debugSensorln(range_right);
    
    delay(4);

    digitalWrite(PIN_SIGNAL, HIGH); 
    time_last_cycle = time_now;
  }
}

void setup() {
  Serial.begin(115200);

  // pin settings
  pinMode(PIN_LED_START_GREEN, OUTPUT);
  pinMode(PIN_LED_START_RED, OUTPUT);
  pinMode(PIN_LED_CIRCLE, OUTPUT);
  pinMode(PIN_LED_HOME, OUTPUT);
  pinMode(PIN_SIGNAL, OUTPUT);

  digitalWrite(PIN_SIGNAL, HIGH); 
  digitalWrite(PIN_LED_START_GREEN, HIGH);
  digitalWrite(PIN_LED_START_RED, HIGH);
  digitalWrite(PIN_LED_CIRCLE, HIGH);
  digitalWrite(PIN_LED_HOME, HIGH);
}

void loop(){
  // read range sensors
  readRangeSensors(range_right, range_front, range_left);

  // process commands 
  if (Serial.available() > 0)
  {
    message = Serial.readStringUntil('\n');
    command = message.substring(0, message.indexOf(' ')).toInt();
    value = message.substring(message.indexOf(' ')+1);

    switch (command)
    {

      case CMD_GET_RANGE_VALUES:{
        Serial.println(String(range_right) + " " + String(range_front) + " " + String(range_left));

        debugComln("(" + String(CMD_GET_RANGE_VALUES) + ")" + " right " + " front " + " left");
        break;
      }

      case CMD_GET_BUTTON_STATES:{
        button_state_circle = !digitalRead(PIN_BUTTON_CIRCLE);
        button_state_start = !digitalRead(PIN_BUTTON_START);
        button_state_home = !digitalRead(PIN_BUTTON_HOME);
        
        Serial.println(String((button_state_circle) ? "1" : "0") + " "
                     + String((button_state_start)  ? "1" : "0") + " "
                     + String((button_state_home)   ? "1" : "0") + " ");

        debugComln("(" + String(CMD_GET_BUTTON_STATES) + ")" + " circle " + " start " + " home");
        break;
      }

      case CMD_GET_LIMIT_SWITCH_STATES:{
        limit_switch_state_right = digitalRead(PIN_LIMIT_SWITCH_RIGHT);
        limit_switch_state_left = digitalRead(PIN_LIMIT_SWITCH_LEFT);
        
        Serial.println(String((limit_switch_state_right) ? "1" : "0") + " "
                     + String((limit_switch_state_left)  ? "1" : "0") + " ");

        debugComln("(" + String(CMD_GET_LIMIT_SWITCH_STATES) + ")" + " right " + " left" );
        break;
      }

      case CMD_SET_LEDS:{
        debugCom(value[0]);
        debugCom(" ");
        debugCom(value[2]);
        debugCom(" ");
        debugCom(value[4]);
        debugCom(" ");
        debugCom(value[6]);
        
        if(value.length() == 7){
          LED_state_circle = (value[0] == '1');
          LED_state_start_green = (value[2] == '1');
          LED_state_start_red = (value[4] == '1');
          LED_state_home = (value[6] == '1');

          digitalWrite(PIN_LED_CIRCLE, !LED_state_circle);
          digitalWrite(PIN_LED_START_GREEN, !LED_state_start_green);
          digitalWrite(PIN_LED_START_RED, !LED_state_start_red);
          digitalWrite(PIN_LED_HOME, !LED_state_home);

          debugComln("(" + String(CMD_GET_BUTTON_STATES) + ")" + " LED circle: " +  String(LED_state_circle)
                                                               + " LED start green: " +  String(LED_state_start_green)
                                                               + " LED start red: " +  String(LED_state_start_red)
                                                               + " LED home: " +  String(LED_state_home));
        }
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