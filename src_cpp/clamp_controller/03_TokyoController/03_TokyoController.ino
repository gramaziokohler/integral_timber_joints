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

#include <DCMotor.h>        // ITJ library
#include <Encoder.h>        // Encoder library from PJRC.COM, LLC - Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <MotorController.h>    //New Class in Development

#include "BufferedSerial.h"

//MOTOR_CONTROL_PRINTOUT (if defined) will print motor control error during motor movements.
// #define MOTOR_CONTROL_PRINTOUT

//SERIAL_MASTER_INIT_TALK_ONLY (if defined) will ensure the controller do not initiate any communication.
//	The controller will act as a slave node and only talk back when the master initiate a message.
//	This is crucial in a master managed time share network to avoid bus contention.
//	This feature can be disabled for debug use when the controller is connected directly to a PC.
//#define SERIAL_MASTER_INIT_TALK_ONLY



//Pins for Motor Driver M1
const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to

const double m1_kp = 0.005;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kp = 0.040
const double m1_ki = 0.200;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_ki = 0.200
const double m1_kd = 0.0002;                // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kd = 0.0002
const double m1_accel = 3000;               // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_accel = 3000

//Pins for Motor Driver M2
const uint8_t m2_driver_ena_pin = 10;          // Reserved Pin
const uint8_t m2_driver_in1_pin = 12;          // Reserved Pin
const uint8_t m2_driver_in2_pin = 11;          // Reserved Pin

//Pins for Motor feedback M1 / M2
const uint8_t m1_encoder1_pin = 2;             // Motor encoder channel C1 (typically the Interrupt pin)
const uint8_t m1_encoder2_pin = 3;             // Motor encoder channel C2
const uint8_t m2_encoder1_pin = 4;             // Reserved Pin
const uint8_t m2_encoder2_pin = 5;             // Reserved Pin

//Pins for Homing Switch
const uint8_t m1_home_pin = A5;                 // (Never use A6 A7)
const uint8_t m2_home_pin = A4;                 // Reserved Pin

const double m1_home_position_step = -1000;

//Pins for Battery Monitor
const uint8_t battery_monitor_pin = A7;

// ---- END OF MODIFIABLE SETTINGS - Do not modify below ----

// Initialize motion control objects
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
Encoder Encoder1(m1_encoder1_pin, m1_encoder2_pin);
MotorController MotorController1(&Motor1, &Encoder1, m1_kp, m1_ki, m1_kd, m1_accel, 10, 50, false, false);

//Variables for serial communication
byte incomingByte;
char status_string[61];

//Variables for battery monitor
int batt_value;

//Variables for profiling
unsigned long profile_start_micros = 0;
unsigned long profile_end_micros = 0;

//

BufferedSerial bufferedSerial(1);

// the setup function runs once when you press reset or power the board
void setup() {

    // Initialize Serial
    bufferedSerial.serialInit();

    // Initial Homing Parameters
    MotorController1.setHomingParam(m1_home_pin, HIGH, m1_home_position_step);

    //Initialize battery monitor pin
    pinMode(battery_monitor_pin, INPUT);

}

void loop() {
    //The main loop implements quasi-time sharing task management.
    //This require all the subroutines to execute in relatively short time.
    //Long subroutine such as Serial prints should be used with cuation.

    
    // Run motor control
    if (MotorController1.run()) {
        Serial.print(F("CurPos: "));
        Serial.print((long) MotorController1.currentPosition());
        Serial.print(F(" Error: "));
        Serial.print((long)MotorController1.currentTarget() - (long)MotorController1.currentPosition());
        Serial.print(F(" PWM: "));
        Serial.println(MotorController1.currentMotorPowerPercentage());
    }

    // Run battery report
    run_batt_monitor();

    // Handle serial command input
    if (bufferedSerial.available()) {
        const char * command = bufferedSerial.read();
        Serial.println(command);
        run_command_handle(command);
    }

 
}

// Serial and command parsing

void run_command_handle(const char * command) {
    
    if (*command == 'h') {
        Serial.println("Command Home : Homing");
        MotorController1.home(true, 500);
    }
    
    if (*command == '?') {
        Serial.println(get_current_status_string());
    }

    if (*command == 'v') {
        double velocity = atof(command + 1);
        Serial.print("Set Velocity: ");
        Serial.println(velocity);
        MotorController1.setDefaultVelocity(velocity);
    }

    if (*command == 'g') {
        long target_position_step = atol(command + 1);
        Serial.print("Goto Position:" );
        Serial.println(target_position_step);
        MotorController1.moveToPosition(target_position_step);
    }

    //if (*command == '+') {
    //    long target_position_step = MotorController1.currentPosition() + atol(command + 1);
    //    Serial.print("Increment Position:");
    //    Serial.println(target_position_step);
    //    MotorController1.moveToPosition(target_position_step);
    //}

    //if (*command == '-') {
    //    long target_position_step = MotorController1.currentPosition() - atol(command + 1);
    //    Serial.print("Decrement Position:");
    //    Serial.println(target_position_step);
    //    MotorController1.moveToPosition(target_position_step);
    //}

    if (*command == 's') {
        Serial.println(F("Command S : Stop Now"));
        MotorController1.stop();
    }

}


// Battery Monitor - To be separated into its own class and file.
void run_batt_monitor() {
    {
        const unsigned long MONITOR_PEIROD_MILLIS = 500;
        static unsigned long next_run_time = 0;
        if (millis() > next_run_time) {
            //profile_start();
            next_run_time = millis() + MONITOR_PEIROD_MILLIS;
            batt_value = analogRead(battery_monitor_pin);
            //profile_end("Batt Monitor Time Taken : ");
        }

    }
}

// Status Reporting
char* get_current_status_string() {
    unsigned int i = 0; // This variable keep count of the output string.
    i += snprintf(status_string + i, 60 - i, "%ld , %u , %i", ((long)MotorController1.currentPosition()), get_status_code(), batt_value);
    return status_string;
}

byte get_status_code() {
    byte code = 0;
    bitWrite(code, 0, (MotorController1.isDirectionExtend()));
    bitWrite(code, 2, (MotorController1.isMotorRunning()));
    bitWrite(code, 4, MotorController1.isHomed());
    return code;
}

