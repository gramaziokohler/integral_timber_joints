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

#include "Direction.h"
#include <EEPROM.h>

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

//Variables for Motion Control
direction m1_direction = EXTEND; // true = Forward
direction m2_direction = EXTEND; // true = Forward
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

    //Initialize motor driver pins
    pinMode(m1_driver_ena_pin, OUTPUT);
    pinMode(m1_driver_in1_pin, OUTPUT);
    pinMode(m1_driver_in2_pin, OUTPUT);

    //Initialize the motor encoder pins 
    pinMode(m1_encoder1_pin, INPUT);
    pinMode(m1_encoder2_pin, INPUT);

    //Initialize battery monitor pin
    pinMode(battery_monitor_pin, INPUT);

}

void loop() {
    //The main loop implements quasi-time sharing task management.
    //This require all the subroutines to execute in relatively short time.
    //Long subroutine such as Serial prints should be used with cuation.


    // Handle serial input and set flags (TODO, Actually ste flags)
    run_serial_handle();

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

void run_motor_1() {
    // Return if the motor state is not running
    if (!m1_running) return;

    // Detect if the motor has finished the move
    if (m1_direction == EXTEND && m1_encoder > m1_ending_encoder) {
        m1_stop();
        #ifndef SERIAL_MASTER_INIT_TALK_ONLY
        Serial.println("M1 Extend Complete, Current Position:");
        Serial.println(m1_encoder / step_per_mm);
        #endif // !SERIAL_MASTER_INIT_TALK_ONLY
        return;
    }
    if (m1_direction == RETRACT && m1_encoder < m1_ending_encoder) {
        m1_stop();
        #ifndef SERIAL_MASTER_INIT_TALK_ONLY
        Serial.print("M1 Retract Complete, Current Position:");
        Serial.println(m1_encoder / step_per_mm);
        #endif // !SERIAL_MASTER_INIT_TALK_ONLY
        return;
    }

    // Compute current waypoint
    long delta_t = millis() - m1_starting_time;
    double current_waypoint_1_encoder_delta = (delta_t * m1_speed_step_per_millis);
    double current_waypoint_1_encoder = m1_starting_encoder + (delta_t * m1_speed_step_per_millis);



    //Compute Difference and set Motor Speed
    long m1_actual_encoder_delta = m1_encoder - m1_starting_encoder;
    double m1_abs_encoder_error = abs(current_waypoint_1_encoder_delta) - abs(m1_actual_encoder_delta);
    // Error is positive If the motor is too slow
    if (m1_abs_encoder_error > 2) {
        m1_set_power(255);
    } else if (m1_abs_encoder_error > 0) {
        //m1_set_power(150);
        m1_set_power(255);
    } else if (m1_abs_encoder_error > -2) {
        //m1_set_power(20);
        m1_set_power(0);
    } else {
        m1_set_power(0);
    }

    //Detect jam based on deviation
    if (m1_abs_encoder_error > encoder_error_before_stop || m1_abs_encoder_error < -encoder_error_before_stop) {
        m1_stop();
        //m2_stop();
        Serial.println("Motor 1 Deviation out of range. All motors will stop.");
        Serial.println(get_current_status_string());
        #ifndef SERIAL_MASTER_INIT_TALK_ONLY
        #endif // !SERIAL_MASTER_INIT_TALK_ONLY
    }

    //DEBUG Monitiring
    #ifndef SERIAL_MASTER_INIT_TALK_ONLY
    #ifdef MOTOR_CONTROL_PRINTOUT
    Serial.print("Motor1Running;");
    Serial.print(delta_t);
    Serial.print(";");
    Serial.print(m1_actual_encoder_delta);
    Serial.print(";");
    Serial.print(m1_abs_encoder_error);
    if (m1_abs_encoder_error > 10 || m1_abs_encoder_error < -10) {
        Serial.print(" DeviationWarning");
    }
    Serial.print('\n');
    #endif // MOTOR_CONTROL_PRINTOUT
    #endif // !SERIAL_MASTER_INIT_TALK_ONLY

}


void set_m1_target(long position_mm, double speed_mm_per_sec) {
    //  m1_starting_encoder
    //	m1_ending_encoder 
    //  m1_starting_time
    //  m1_speed_step_per_millis
    m1_starting_encoder = m1_encoder;
    m1_ending_encoder = position_mm * step_per_mm;

    if (m1_ending_encoder == m1_starting_encoder) return; // Exit the function if the target is the current position_mm.
    if (m1_ending_encoder > m1_starting_encoder) {
        m1_set_direction(EXTEND);
        m1_speed_step_per_millis = abs(speed_mm_per_sec) * step_per_mm / 1000.0;
    } else {
        m1_set_direction(RETRACT);
        m1_speed_step_per_millis = -abs(speed_mm_per_sec) * step_per_mm / 1000.0;
    }
    m1_starting_time = millis();
    //m1_set_power(255); //Set Initial Power (this also raises the Running flag)
    m1_start_full_power();
}

void run_serial_handle() {
    static boolean escaping_state = false;

    if (Serial.available() > 0) {
        profile_start();
        // read the incoming byte:
        incomingByte = Serial.read();

        // Parse Escaping Character
        if (incomingByte == '(') {
            escaping_state = true;
        }

        // Parse Unescaping Character
        if (incomingByte == ')' || incomingByte == '\n' || incomingByte == '\r') {
            escaping_state = false;
        }

        // Continue to read all character during escaping state //This reduce the time to precess unrelated comment message.
        if (escaping_state) {
            while (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                if (incomingByte == ')' || incomingByte == '\n' || incomingByte == '\r') {
                    escaping_state = false;
                }
            }
            return;
        }

        // Parse Input
        if (incomingByte == 'h') {
            Serial.println("Command Home : Home by extending");
            motors_home();
        }
        // Parse Input
        if (incomingByte == '?') {
            Serial.println(get_current_status_string());
        }
        // Parse Input
        if (incomingByte == '+') {
            Serial.println("Command + : Increment 1");
            //set_m1_target(105, axis_speed);
            m1_set_direction(EXTEND);
            m1_start_full_power();
            delay(200);
            m1_stop();
        }
        // Parse Input
        if (incomingByte == '-') {
            Serial.println("Command - : Decrement 1");
            /*set_m1_target(2, axis_speed);*/
            m1_set_direction(RETRACT);
            m1_start_full_power();
            delay(200);
            m1_stop();
        }
        // Parse Input
        if (incomingByte == 's') {
            Serial.println("Command s : Immediate Stop");
            m1_stop();
            //m2_stop();
        }

        //if (incomingByte == 'e') {
        //    //Extend the longer arm one mm and the shorter arm to match 
        //    double longer_arm_position = max(m1_encoder, m2_encoder) / step_per_mm;
        //    double new_position = longer_arm_position + 0.5;
        //    Serial.print(F("Command e - Syncro Extend 0.5mm NewPos:"));
        //    Serial.println(new_position);
        //    set_m1_target(new_position, axis_speed);
        //    set_m2_target(new_position, axis_speed);
        //}
        //if (incomingByte == 'r') {
        //    //Extend the longer arm one mm and the shorter arm to match 
        //    double longer_arm_position = min(m1_encoder, m2_encoder) / step_per_mm;
        //    double new_position = longer_arm_position - 0.5;
        //    Serial.print(F("Command r - Syncro Retract 0.5mm NewPos:"));
        //    Serial.println(new_position);
        //    set_m1_target(new_position, axis_speed);
        //    set_m2_target(new_position, axis_speed);
        //}

        // Settings Commands
        if (incomingByte == '$') {
            while (!Serial.available());
            byte commandByte = Serial.read();
            // $0 Set Axis Speed
            if (commandByte == '0') {
                delayMicroseconds(10);
                double speed = (double)Serial.parseFloat();
                set_axis_speed(speed);
                Serial.print(F("Setting 0 - Axis Speed = "));
                Serial.println(speed);
            }
            // $1 Set m1 default home position
            if (commandByte == '1') {
                delayMicroseconds(10);
                double position = (double)Serial.parseFloat();
                set_m1_default_home_position(position);
                Serial.print(F("Setting 1 - m1_default_home_position = "));
                Serial.println(m1_default_home_position);
            }
            // $2 Set m2 default home position
            if (commandByte == '2') {
                delayMicroseconds(10);
                double position = (double)Serial.parseFloat();
                set_m2_default_home_position(position);
                Serial.print(F("Setting 2 - m2_default_home_position = "));
                Serial.println(m2_default_home_position);
            }
        }

        //// Test Commands
        //if (incomingByte == 't') {
        //    while (!Serial.available());
        //    byte commandByte = Serial.read();
        //    if (commandByte == '0') {
        //        motor_full_retract_test(M1);
        //        motor_full_retract_test(M2);
        //    }
        //    if (commandByte == '1') {
        //        motor_full_retract_test(M1);
        //    }
        //    if (commandByte == '2') {
        //        motor_full_retract_test(M2);
        //    }
        //}
        // TODO: Still need to implement a char by char reader which can return me a whole line.
        profile_end("Serial Handle micros() : ");
    }
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
