/*
 Name:		_03_TokyoController.ino
 Created:	3/9/2020 7:00:41 PM
 Author:	leungp

 This controller ocntinues the development from 01_SerialCommandController

 The controller is reading Serial Port Commands. Supported commands:
 h - Home
 ? - Status
 g10 - goto10
 s - stop
 + - increment
 - - decrement

 The controller now uses a PID controller for the motors for position control
 to follow a motion profile.


*/

// The include statements are all in <angle brackets> because compiler need to look in the include folder for these files.
// For example "Double Quote" won't work for Encoder.h because it has folder structure in its nested imports.
// All libraries should ideally be located in libraries include folder.
// * ITJ libraries are developed in the integral_timber_joints project. Located in \integral_timber_joints\src_cpp\libraries

#include <EEPROM.h>         // Arduino default library for accessing EEPROM

#include "MotorController.h"    //New Class in Development
#include <DCMotor.h>        // ITJ library
#include <Encoder.h>        // Encoder library from PJRC.COM, LLC - Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html

#include <PID_v1.h>         // Arduino PID Library - Version 1.2.1 by Brett Beauregard https://github.com/br3ttb/Arduino-PID-Library/
#include <MotionProfile.h>  // ITJ library


//MOTOR_CONTROL_PRINTOUT (if defined) will print motor control error during motor movements.
// #define MOTOR_CONTROL_PRINTOUT

//SERIAL_MASTER_INIT_TALK_ONLY (if defined) will ensure the controller do not initiate any communication.
//	The controller will act as a slave node and only talk back when the master initiate a message.
//	This is crucial in a master managed time share network to avoid bus contention.
//	This feature can be disabled for debug use when the controller is connected directly to a PC.
//#define SERIAL_MASTER_INIT_TALK_ONLY


//Settings for motion system
double m1_default_home_position = 110.0; //TODO: Real code should use value read fom EEPROM
double m2_default_home_position = 108.0;

const double step_per_mm = 82.28571428571428571428; //40mm -> 40.9
const double encoder_error_before_stop = 15;

//Pins for Motor Driver M1
const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to

const double m1_kp = 0.040;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile 
const double m1_ki = 0.200;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile 
const double m1_kd = 0.0002;                // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile 
const double m1_accel = 3000;               // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile 

//Pins for Motor Driver M2
//const uint8_t m2_driver_ena_pin = 10;          // the pin the motor driver ENA2 is attached to (PWM Pin)
//const uint8_t m2_driver_in1_pin = 12;          // the pin the motor driver IN3 is attached to
//const uint8_t m2_driver_in2_pin = 11;          // the pin the motor driver IN4 is attached to

//Pins for Motor feedback M1 / M2
const uint8_t m1_encoder1_pin = 2;             // Motor encoder channel C1 (typically the Interrupt pin)
const uint8_t m1_encoder2_pin = 3;             // Motor encoder channel C2

//Pins for Homing Switch
const uint8_t m1_home_pin = A6;

//Pins for Battery Monitor
const uint8_t battery_monitor_pin = A7;

// ---- END OF MODIFIABLE SETTINGS - Do not modify below ----

// Initialize motion control objects
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
Encoder Encoder1(m1_encoder1_pin, m1_encoder2_pin);
MotorController MotorController1(&Motor1, &Encoder1, m1_kp, m1_ki, m1_kd, m1_accel);

// Variables for Motion Control
Direction m1_direction = EXTEND; // true = Forward
Direction m2_direction = EXTEND; // true = Forward
volatile long m1_encoder = 0;
volatile long m2_encoder = 0;
double axis_speed = 2.0;
boolean axis_homed = false;

//State variable for Motion Control
boolean m1_running = false;
long m1_starting_time = 0;
long m1_starting_encoder = 0;
long m1_ending_encoder = 0;
double m1_speed_step_per_millis = 0.0; //mm per millis

//Variables for serial communication
byte incomingByte;
char status_string[61];

//Variables for battery monitor
int batt_value;

//Variables for profiling
unsigned long profile_start_micros = 0;
unsigned long profile_end_micros = 0;


// the setup function runs once when you press reset or power the board
void setup() {

    //Setup Serial Port
    Serial.begin(115200);
    Serial.println(F("(Motor controller startup)"));

    //Initialize battery monitor pin
    pinMode(battery_monitor_pin, INPUT);

}

void loop() {
    //The main loop implements quasi-time sharing task management.
    //This require all the subroutines to execute in relatively short time.
    //Long subroutine such as Serial prints should be used with cuation.


    // Handle serial input and set flags (TODO, Actually ste flags)
    // run_serial_handle();

    // Run motor control
    //run_motor_1();

    // Run battery report
    run_batt_monitor();

}

void run_batt_monitor() {
    {
        
        const unsigned long MONITOR_PEIROD_MILLIS = 500;
        static unsigned long next_report_time = 0;
        if (millis() > next_report_time) {
            //profile_start();
            next_report_time = millis() + MONITOR_PEIROD_MILLIS;
            batt_value = analogRead(battery_monitor_pin);
            //profile_end("Batt Monitor Time Taken : ");
        }

    }
    #ifndef SERIAL_MASTER_INIT_TALK_ONLY
    {
        const unsigned long REOPRTING_PEIROD_MILLIS = 500;
        static unsigned long next_report_time = 0;
        if (millis() > next_report_time) {
            next_report_time = millis() + REOPRTING_PEIROD_MILLIS;
            Serial.print("Batt Level:");
            Serial.println(batt_value);
        }
    }
    #endif // !SERIAL_MASTER_INIT_TALK_ONLY
}

char* get_current_status_string() {
    unsigned int i = 0; // This variable keep count of the output string.
    double step_per_mm_div100 = step_per_mm / 100;
    //m1 and m2 position
    i += snprintf(status_string + i, 60 - i, "%i,%i,%i,%i", (long)(m1_encoder / step_per_mm_div100), (long)(m2_encoder / step_per_mm_div100), get_status_code(), batt_value);
    return status_string;
}

char* get_current_status_string_old() {
    unsigned int i = 0; // This variable keep count of the output string.
    double step_per_mm_div100 = step_per_mm / 100;
    //m1 and m2 position
    i += snprintf(status_string + i, 60 - i, "%i,", (long)(m1_encoder / step_per_mm_div100));
    i += snprintf(status_string + i, 60 - i, "%i,", (long)(m2_encoder / step_per_mm_div100));
    //status code
    i += snprintf(status_string + i, 60 - i, "%i,", get_status_code());
    //battery monitor value
    i += snprintf(status_string + i, 60 - i, "%i", batt_value);
    return status_string;
}

byte get_status_code() {
    byte code = 0;
    bitWrite(code, 0, (m1_direction == EXTEND));
    //bitWrite(code, 1, (m2_direction == EXTEND));
    bitWrite(code, 2, (m1_running));
    //bitWrite(code, 3, (m2_running));
    bitWrite(code, 4, (axis_homed));
    return code;
}

void run_command_handle(char *command) {
    // Parse Input
    if (*command == 'h') {
        Serial.println("Command Home : Home by extending");
        //motors_home();
    }
    // Parse Input
    if (*command == '?') {
        Serial.println(get_current_status_string());
    }
    // Parse Input
    if (*command == '+') {
        Serial.println("Command + : Increment 1");
        //set_m1_target(105, axis_speed);
        //m1_set_direction(EXTEND);
        //m1_start_full_power();
        //delay(200);
        //m1_stop();
    }
    // Parse Input
    if (*command == '-') {
        Serial.println("Command - : Decrement 1");
        /*set_m1_target(2, axis_speed);*/
        //m1_set_direction(RETRACT);
        //m1_start_full_power();
        //delay(200);
        //m1_stop();
    }
    // Parse Input
    if (*command == 's') {
        //Serial.println("Command s : Immediate Stop");
        //MotorController1.stop();
        //m2_stop();
    }

}
