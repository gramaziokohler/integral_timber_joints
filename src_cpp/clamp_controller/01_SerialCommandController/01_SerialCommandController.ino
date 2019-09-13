/*
 Name:		_01_SerialCommandController.ino
 Created:	7/19/2019 3:22:34 PM
 Author:	leungp
*/

// TODO
// More test is needed to see if a better PID control would be better than the current BangBang control.
// At the moment, m1_set_power() and m2_set_power() is modified to do Digital Out instead of PWN out
// This is in the fear that PWM frequency (Default 976.56 Hz) may cause encoder to be noisy.

// TODO
// Homing cycle is currently blocking and will not respond to further serial commands until homing is complete.
// This should be non-blocking and allow serial commands to ping if the homing is completed.

// TODO
// Add warning message if the EEPROM have not been initialized after being flashed.

#include "Direction.h"
#include <HX711.h>
#include <EEPROM.h>

//MOTOR_CONTROL_PRINTOUT (if defined) will print motor control error during motor movements.
// #define MOTOR_CONTROL_PRINTOUT

//SERIAL_MASTER_INIT_TALK_ONLY (if defined) will ensure the controller do not initiate any communication.
//	The controller will act as a slave node and only talk back when the master initiate a message.
//	This is crucial in a master managed time share network to avoid bus contention.
//	This feature can be disabled for debug use when the controller is connected directly to a PC.
#define SERIAL_MASTER_INIT_TALK_ONLY

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
const uint8_t m2_driver_ena_pin = 10;          // the pin the motor driver ENA2 is attached to (PWM Pin)
const uint8_t m2_driver_in1_pin = 12;          // the pin the motor driver IN3 is attached to
const uint8_t m2_driver_in2_pin = 11;          // the pin the motor driver IN4 is attached to

//Pins for Motor feedback M1 / M2
const uint8_t m1_hall_sensor_pin = 3;             // the pin the linear actuator M1 hall effect sensor is attached to
const uint8_t m2_hall_sensor_pin = 2;             // the pin the linear actuator M1 hall effect sensor is attached to

//Pins for Battery Monitor
const uint8_t battery_monitor_pin = A7;

//Pins for Load Cell HX711
const uint8_t lc1_sda_pin = A1;
const uint8_t lc1_sck_pin = A2;
const uint8_t lc2_sda_pin = A4;
const uint8_t lc2_sck_pin = A3;

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

boolean m2_running = false;
long m2_starting_time = 0;
long m2_starting_encoder = 0;
long m2_ending_encoder = 0;
double m2_speed_step_per_millis = 0.0; //mm per millis

//Variables for serial communication
byte incomingByte;
char status_string[61];

//Variables for load cells
HX711 lc1;
HX711 lc2;
float lc1_scale = 1.0; // Calibration value for the load cell
float lc2_scale = 1.0; // Calibration value for the load cell
float lc1_value; // Stores the up-to-date value of the load cell
float lc2_value; // Stores the up-to-date value of the load cell

//Variables for battery monitor
int batt_value;

void setup() {
	//Load EEPROM Settings
	load_eeprom_setting();

	//Setup Serial Port
	Serial.begin(115200);
	Serial.println(F("(Motor controller startup)"));
	
	//Initialize the motor encoder intterrupt 
	pinMode(m1_hall_sensor_pin, INPUT);
	pinMode(m2_hall_sensor_pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(m1_hall_sensor_pin), encoder_trigger_1, RISING); // Arduino pin 2
	attachInterrupt(digitalPinToInterrupt(m2_hall_sensor_pin), encoder_trigger_2, RISING); // Arduino pin 2

	//Initialize motor driver pins
	pinMode(m1_driver_ena_pin, OUTPUT);
	pinMode(m1_driver_in1_pin, OUTPUT);
	pinMode(m1_driver_in2_pin, OUTPUT);
	pinMode(m2_driver_ena_pin, OUTPUT);
	pinMode(m2_driver_in1_pin, OUTPUT);
	pinMode(m2_driver_in2_pin, OUTPUT);

	//Initialize battery monitor pin
	pinMode(battery_monitor_pin, INPUT);

	//Initialize load cell pins
	lc1.begin(lc1_sda_pin, lc1_sck_pin, 64U); //begin() will configure pinMode.
	lc2.begin(lc2_sda_pin, lc2_sck_pin, 64U); //begin() will configure pinMode.

	//Homing routine
	motors_home();

	//Tare the load cell at home position
	lc1.tare(4U);
	lc2.tare(4U);
}


// the loop routine runs over and over again forever:
void loop() {
	//The main loop implements quasi-time sharing task management.
	//This require all the subroutines to execute in relatively short time.
	//Long subroutine such as Serial prints should be used with cuation.

	// Handle serial input and set flags (TODO, Actually ste flags)
	run_serial_monitor();

	// Perform heartbeat serial report
	//handle_heartbeat();

	// Run motor control
	run_motor_1();
	run_motor_2();

	// Run battery report
	run_batt_monitor();

	// Run Loadcell report
	run_loadcell_monitor();

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
	}
	else if (m1_abs_encoder_error > 0) {
		//m1_set_power(150);
		m1_set_power(255);
	}
	else if (m1_abs_encoder_error > -2) {
		//m1_set_power(20);
		m1_set_power(0);
	}
	else {
		m1_set_power(0);
	}

	//Detect jam based on deviation
	if (m1_abs_encoder_error > encoder_error_before_stop || m1_abs_encoder_error < -encoder_error_before_stop) {
		m1_stop();
		m2_stop();
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

void run_motor_2() {
	// Return if the motor state is not running
	if (!m2_running) return;

	// Detect if the motor has finished the move
	if (m2_direction == EXTEND && m2_encoder > m2_ending_encoder) {
		m2_stop();
		#ifndef SERIAL_MASTER_INIT_TALK_ONLY
		Serial.print("M2 Extend Complete, Current Position:");
		Serial.println(m2_encoder / step_per_mm);
		#endif // !SERIAL_MASTER_INIT_TALK_ONLY
		return;
	}
	if (m2_direction == RETRACT && m2_encoder < m2_ending_encoder) {
		m2_stop();
		#ifndef SERIAL_MASTER_INIT_TALK_ONLY
		Serial.print("M2 Retract Complete, Current Position:");
		Serial.println(m2_encoder / step_per_mm);
		#endif // !SERIAL_MASTER_INIT_TALK_ONLY
		return;
	}

	// Compute current waypoint
	long delta_t = millis() - m2_starting_time;
	double current_waypoint_2_encoder_delta = (delta_t * m2_speed_step_per_millis);
	double current_waypoint_2_encoder = m2_starting_encoder + (delta_t * m2_speed_step_per_millis);



	//Compute Difference and set Motor Speed
	long m2_actual_encoder_delta = m2_encoder - m2_starting_encoder;
	double m2_abs_encoder_error = abs(current_waypoint_2_encoder_delta) - abs(m2_actual_encoder_delta);
	// Error is positive If the motor is too slow
	if (m2_abs_encoder_error > 2) {
		m2_set_power(255);
	}
	else if (m2_abs_encoder_error > 0) {
		// m2_set_power(150);
		m2_set_power(255);
	}
	else if (m2_abs_encoder_error > -2) {
		//m2_set_power(20);
		m2_set_power(0);
	}
	else {
		m2_set_power(0);
	}

	//Detect jam based on deviation
	if (m2_abs_encoder_error > encoder_error_before_stop || m2_abs_encoder_error < -encoder_error_before_stop) {
		m1_stop();
		m2_stop();
		Serial.println("Motor 2 Deviation out of range. All motors will stop.");
        Serial.println(get_current_status_string());
		#ifndef SERIAL_MASTER_INIT_TALK_ONLY
		#endif // !SERIAL_MASTER_INIT_TALK_ONLY
	}

	//DEBUG Monitiring
	#ifndef SERIAL_MASTER_INIT_TALK_ONLY
	#ifdef MOTOR_CONTROL_PRINTOUT
	Serial.print("Motor2Running;");
	Serial.print(delta_t);
	Serial.print(";");
	Serial.print(m2_actual_encoder_delta);
	Serial.print(";");
	Serial.print(m2_abs_encoder_error);
	if (m2_abs_encoder_error > 10 || m2_abs_encoder_error < -10) {
		Serial.print(" DeviationWarning");
	}
	Serial.print('\n');
	#endif // MOTOR_CONTROL_PRINTOUT
	#endif // !SERIAL_MASTER_INIT_TALK_ONLY
}

void run_batt_monitor() {
	{
		const unsigned long MONITOR_PEIROD_MILLIS = 500;
		static unsigned long next_report_time = 0;
		if (millis() > next_report_time) {
			next_report_time = millis() + MONITOR_PEIROD_MILLIS;
			batt_value = analogRead(battery_monitor_pin);
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

void run_loadcell_monitor() {
	// Check if load cell has new data. This happens approximately every 0.1 sec
	if (lc1.is_ready()) lc1_value = lc1.get_units();
	if (lc2.is_ready()) lc2_value = lc2.get_units();

	// Report load cell values
	#ifndef SERIAL_MASTER_INIT_TALK_ONLY
	const unsigned long REOPRTING_PEIROD_MILLIS = 500;
	static unsigned long next_report_time = 0;
	if (millis() > next_report_time) {
		next_report_time = millis() + REOPRTING_PEIROD_MILLIS;
		Serial.print(F("Loadcell Value:"));
		Serial.print(lc1_value);
		Serial.print(F(","));
		Serial.println(lc2_value);
	}
	#endif // !SERIAL_MASTER_INIT_TALK_ONLY
}

// Sets the motion profile of the move. The following variables are set:
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
	}
	else {
		m1_set_direction(RETRACT);
		m1_speed_step_per_millis = - abs(speed_mm_per_sec) * step_per_mm / 1000.0;
	}
	m1_starting_time = millis();
	//m1_set_power(255); //Set Initial Power (this also raises the Running flag)
	m1_start_full_power();
}

void set_m2_target(long position, double speed_mm_per_sec) {
	m2_starting_encoder = m2_encoder;
	m2_ending_encoder = position * step_per_mm;

	if (m2_ending_encoder == m2_starting_encoder) return; // Exit the function if the target is the current position_mm.
	if (m2_ending_encoder > m2_starting_encoder) {
		m2_set_direction(EXTEND);
		m2_speed_step_per_millis = abs(speed_mm_per_sec) * step_per_mm / 1000.0;
	}
	else {
		m2_set_direction(RETRACT);
		m2_speed_step_per_millis = -abs(speed_mm_per_sec) * step_per_mm / 1000.0;
	}
	m2_starting_time = millis();
	//m2_set_power(255); //Set Initial Power (this also raises the Running flag)
	m2_start_full_power();

}

void run_serial_monitor() {
	static boolean escaping_state = false;

	if (Serial.available() > 0) {
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
			Serial.println("Command + : Expend to 105");
			set_m1_target(105, axis_speed);
			set_m2_target(105, axis_speed);
		}
		// Parse Input
		if (incomingByte == '-') {
			Serial.println("Command - : Retract to 00");
			set_m1_target(2, axis_speed);
			set_m2_target(2, axis_speed);
		}
		// Parse Input
		if (incomingByte == 's') {
			Serial.println("Command s : Immediate Stop");
			m1_stop();
			m2_stop();
		}

		// Positional Commands - go to position
		if (incomingByte == '9') {
			Serial.println("Command 9 : Goto 90");
			set_m1_target(90, axis_speed);
			set_m2_target(90, axis_speed);
		}
		if (incomingByte == '8') {
			Serial.println("Command 8 : Goto 80");
			set_m1_target(80, axis_speed);
			set_m2_target(80, axis_speed);
		}
		if (incomingByte == '7') {
			Serial.println("Command 7 : Goto 70");
			set_m1_target(70, axis_speed);
			set_m2_target(70, axis_speed);
		}
		if (incomingByte == '6') {
			Serial.println("Command 6 : Goto 60");
			set_m1_target(60, axis_speed);
			set_m2_target(60, axis_speed);
		}
		if (incomingByte == '5') {
			Serial.println("Command 5 : Goto 50");
			set_m1_target(50, axis_speed);
			set_m2_target(50, axis_speed);
		}
		if (incomingByte == '4') {
			Serial.println("Command 4 : Goto 40");
			set_m1_target(40, axis_speed);
			set_m2_target(40, axis_speed);
		}
		if (incomingByte == '3') {
			Serial.println("Command 3 : Goto 30");
			set_m1_target(30, axis_speed);
			set_m2_target(30, axis_speed);
		}
		if (incomingByte == '2') {
			Serial.println("Command 2 : Goto 20");
			set_m1_target(20, axis_speed);
			set_m2_target(20, axis_speed);
		}
		if (incomingByte == '1') {
			Serial.println("Command 1 : Goto 10");
			set_m1_target(10, axis_speed);
			set_m2_target(10, axis_speed);
		}
		if (incomingByte == '0') {
			Serial.println("Command 0 : Goto 00");
			set_m1_target(0, axis_speed);
			set_m2_target(0, axis_speed);
		}
		if (incomingByte == 'e') {
			//Extend the longer arm one mm and the shorter arm to match 
			double longer_arm_position = max(m1_encoder, m2_encoder) / step_per_mm;
			double new_position = longer_arm_position + 0.5;
            Serial.print(F("Command e - Syncro Extend 0.5mm NewPos:"));
            Serial.println(new_position);
            set_m1_target(new_position, axis_speed);
			set_m2_target(new_position, axis_speed);
		}
        if (incomingByte == 'r') {
            //Extend the longer arm one mm and the shorter arm to match 
            double longer_arm_position = min(m1_encoder, m2_encoder) / step_per_mm;
            double new_position = longer_arm_position - 0.5;
            Serial.print(F("Command r - Syncro Retract 0.5mm NewPos:"));
            Serial.println(new_position);
            set_m1_target(new_position, axis_speed);
            set_m2_target(new_position, axis_speed);
        }

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

		// Test Commands
		if (incomingByte == 't') {
			while (!Serial.available());
			byte commandByte = Serial.read();
			if (commandByte == '0') {
				motor_full_retract_test(M1);
				motor_full_retract_test(M2);
			}
			if (commandByte == '1') {
				motor_full_retract_test(M1);
			}
			if (commandByte == '2') {
				motor_full_retract_test(M2);
			}
		}
		// TODO: Still need to implement a char by char reader which can return me a whole line.

	}
}

void motor_full_retract_test(motor m) {
	//Extend until the system stopped
	motor_run_until_stop(m,EXTEND);

	//Keep track of total number of steps
	unsigned long start_time, stop_time;
	long start_step, stop_step;

	if (m == M1) Serial.println(F("M1 Retract Begin. Please wait"));
	if (m == M2) Serial.println(F("M2 Retract Begin. Please wait"));

	//Test Retraction Direction
	if (m == M1) start_step = m1_encoder;
	if (m == M2) start_step = m2_encoder;
	start_time = millis();
	motor_run_until_stop(m, RETRACT);
	if (m == M1) stop_step = m1_encoder;
	if (m == M2) stop_step = m2_encoder;
	stop_time = millis();

	//Print Result
	if (m == M1) Serial.print(F("M1 Retract Complete. Total Steps:"));
	if (m == M2) Serial.print(F("M2 Retract Complete. Total Steps:"));
	Serial.print(stop_step - start_step);
	Serial.print(F(" Time(ms):"));
	Serial.print(stop_time - start_time);
	Serial.print(F(" Speed(mm/s):"));
	Serial.println((stop_step - start_step) / step_per_mm / (stop_time - start_time) * 1000);
	
	//Test Extension Direction
	if (m == M1) start_step = m1_encoder;
	if (m == M2) start_step = m2_encoder;
	start_time = millis();
	motor_run_until_stop(m, EXTEND);
	if (m == M1) stop_step = m1_encoder;
	if (m == M2) stop_step = m2_encoder;
	stop_time = millis();

	//Print Result
	if (m == M1) Serial.print(F("M1 Extend Complete. Total Steps:"));
	if (m == M2) Serial.print(F("M2 Extend Complete. Total Steps:"));
	Serial.print(stop_step - start_step);
	Serial.print(F(" Time(ms):"));
	Serial.print(stop_time - start_time);
	Serial.print(F(" Speed(mm/s):"));
	Serial.println((stop_step - start_step) / step_per_mm / (stop_time - start_time) * 1000);
}

void motor_run_until_stop(motor m, direction dir) {
	const unsigned long time_without_move_as_stop_sign = 100; //ms

	if (m == M1) {
		int last_step = m1_encoder;

		//Set Speed and direction to retract
		m1_set_direction(dir);
		m1_start_full_power();

		do {
			last_step = m1_encoder;
			delay(time_without_move_as_stop_sign);
		} while (last_step != m1_encoder);

		//Turn off motor just in case.
		m1_stop();
	}

	if (m == M2) {
		int last_step = m2_encoder;

		//Set Speed and direction to retract
		m2_set_direction(dir);
		m2_start_full_power();

		do {
			last_step = m2_encoder;
			delay(time_without_move_as_stop_sign);
		} while (last_step != m2_encoder);

		//Turn off motor just in case.
		m2_stop();
	}
}

char* get_current_status_string() {
	unsigned int i = 0;
	double step_per_mm_div100 = step_per_mm / 100;
	//m1 and m2 position
	i += snprintf(status_string + i, 60 - i, "%i,", (long)(m1_encoder / step_per_mm_div100));
	i += snprintf(status_string + i, 60 - i, "%i,", (long)(m2_encoder / step_per_mm_div100));
	//status code
	i += snprintf(status_string + i, 60 - i, "%i,", get_status_code());
	//loadcell1 and loadcell2 value
	i += snprintf(status_string + i, 60 - i, "%i,", (long)lc1_value);
	i += snprintf(status_string + i, 60 - i, "%i,", (long)lc2_value);
	//battery monitor value
	i += snprintf(status_string + i, 60 - i, "%i", batt_value);
	return status_string;
}

byte get_status_code() {
	byte code = 0;
	bitWrite(code, 0, (m1_direction == EXTEND));
	bitWrite(code, 1, (m2_direction == EXTEND));
	bitWrite(code, 2, (m1_running));
	bitWrite(code, 3, (m2_running));
	bitWrite(code, 4, (axis_homed));
	return code;
}


// Some test Results:

// Battery powered full retract test: (2019-08-02)
// Homing Completed. Steps:9326,9080
// M1 Retract Begin.Please wait
// M1 Retract Complete.Total Steps : -9055 Time(ms) : 24002 Speed(mm / s) : -4.58
// M1 Extend Complete.Total Steps : 9904 Time(ms) : 24502 Speed(mm / s) : 4.91
// M2 Retract Begin.Please wait
// M2 Retract Complete.Total Steps : -8999 Time(ms) : 23203 Speed(mm / s) : -4.71
// M2 Extend Complete.Total Steps : 9004 Time(ms) : 23302 Speed(mm / s) : 4.70
// 12133, 11994, 3, 1219, -95, 850

// Second test right after that, Notice M1 Extend has a weird amount of steps over the expected 9000 steps
// M1 Retract Complete.Total Steps : -9045 Time(ms) : 24103 Speed(mm / s) : -4.56
// M1 Extend Complete.Total Steps : 9580 Time(ms) : 24502 Speed(mm / s) : 4.75
// M2 Retract Begin.Please wait
// M2 Retract Complete.Total Steps : -9003 Time(ms) : 23102 Speed(mm / s) : -4.74
// M2 Extend Complete.Total Steps : 9006 Time(ms) : 23303 Speed(mm / s) : 4.70


// Battery Voltage Monitor Calibration for Prototype Clamp:
// Analog Reading = Voltage
// 892 = 16.1V
// 875 = 15.8V
// 859 = 15.5V
// 842 = 15.2V
// 831 = 15.0V
// 819 = 14.8V (Batt Empty)
