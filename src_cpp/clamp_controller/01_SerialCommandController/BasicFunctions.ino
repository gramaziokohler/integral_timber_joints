//Set Motor Directions
void m1_set_direction(direction dir) {
	m1_direction = dir;
	if (dir == EXTEND) {
		digitalWrite(m1_driver_in1_pin, HIGH);
		digitalWrite(m1_driver_in2_pin, LOW);
	}
	else {
		digitalWrite(m1_driver_in1_pin, LOW);
		digitalWrite(m1_driver_in2_pin, HIGH);
	}
}

void m2_set_direction(direction dir) {
	m2_direction = dir;
	if (dir == EXTEND) {
		digitalWrite(m2_driver_in1_pin, HIGH);
		digitalWrite(m2_driver_in2_pin, LOW);
	}
	else {
		digitalWrite(m2_driver_in1_pin, LOW);
		digitalWrite(m2_driver_in2_pin, HIGH);
	}
}


//Set Motor Power (Positive Value Only)
void m1_set_power(int value) {
	//analogWrite(m1_driver_ena_pin, value);
	
	if(value > 128){
		digitalWrite(m1_driver_ena_pin, HIGH);
	}else{
		digitalWrite(m1_driver_ena_pin, LOW);
	}
}

//Set Motor Power (Positive Value Only)
void m2_set_power(int value) {
	//analogWrite(m2_driver_ena_pin, value);

	if(value > 128){
		digitalWrite(m2_driver_ena_pin, HIGH);
	}else{
		digitalWrite(m2_driver_ena_pin, LOW);
	}
}

void m1_start_full_power() {
	m1_set_power(255);
	m1_running = true;
}

void m1_stop() {
	m1_set_power(0);
	m1_running = false;
}

void m2_start_full_power() {
	m2_set_power(255);
	m2_running = true;
}

void m2_stop() {
	m2_set_power(0);
	m2_running = false;
}

// Set Target linear axis speed in mm/sec
// (Also saves the value to EEPROM)
void set_axis_speed(float speed) {
	axis_speed = speed;
	EEPROM.put(0, speed);
}

// Set M1 Default home position
// (Also saves the value to EEPROM)
void set_m1_default_home_position(float position) {
	m1_default_home_position = position;
	EEPROM.put(10, position);
	//EEPROM
}

// Set M2 Default home position
// (Also saves the value to EEPROM)
void set_m2_default_home_position(float position) {
	m2_default_home_position = position;
	EEPROM.put(20, position);
}

//Load all the $ settings from EEPROM
void load_eeprom_setting() {
	float f = 0.00f;

	// Load Setting $0
	EEPROM.get(0, f);
	set_axis_speed(f);
	// Load Setting $1
	EEPROM.get(10, f);
	set_m1_default_home_position(f);
	// Load Setting $0
	EEPROM.get(20, f);
	set_m2_default_home_position(f);
}

// Homing routine for linear actuator
// Homing drection is to extend (+ve)
// Home detection is by observing that the encoder values are not changing within a 100ms interval
void motors_home() {
	axis_homed = false;
	//Set speed to zero and wait for settling
	m1_stop();
	m2_stop();
	delay(10);
	//Reset Encoder
	m1_encoder = 0;
	m2_encoder = 0;
	double last_encoder_1_value = 0;
	double last_encoder_2_value = 0;

	//Set Direction to extend
	m1_set_direction(EXTEND);
	m2_set_direction(EXTEND);
	//Set speed to Extend Full Speed and wait for a while
	m1_start_full_power();
	m2_start_full_power();
	delay(500);

	#ifndef SERIAL_MASTER_INIT_TALK_ONLY
	#ifdef DEBUG_PRINT
	Serial.print("00 EncoderValue:");
	Serial.print(m1_encoder);
	Serial.print(",");
	Serial.println(m2_encoder);
	#endif
	#endif // !SERIAL_MASTER_INIT_TALK_ONLY

	//Handles if the motor does not move in first attempt
	if (m1_encoder < 10) {
		//Serial.println("Backing Motor 1");
		last_encoder_1_value = m1_encoder;		// Log encoder value
		m1_set_direction(RETRACT);
		delay(500);
	
		#ifndef SERIAL_MASTER_INIT_TALK_ONLY
		#ifdef DEBUG_PRINT
		Serial.print("01 EncoderValue:");
		Serial.print(m1_encoder);
		Serial.print(",");
		Serial.println(m2_encoder);
		#endif
		#endif // !SERIAL_MASTER_INIT_TALK_ONLY

		//Handles if the motor is not moving in second attempt
		if (last_encoder_1_value - m1_encoder < 10) {
			#ifndef SERIAL_MASTER_INIT_TALK_ONLY
			Serial.println("Homing Fail, Motor 1 is not running in either direction");
			#endif // !SERIAL_MASTER_INIT_TALK_ONLY
			m1_stop();
			m2_stop();
			return;
		}
		//After motor is backed, we zero the encoder again and start extending again
		//Set speed to Extend Full Speed and wait for a while
		last_encoder_1_value = m1_encoder;
		m1_set_direction(EXTEND);
		delay(500);
	}

	//Serial.print("10 EncoderValue:");
	//Serial.print(m1_encoder);
	//Serial.print(",");
	//Serial.println(m2_encoder);

	//Handles if the motor does not move in first attempt
	if (m2_encoder < 10) {
		//Serial.println("Backing Motor 2");
		last_encoder_2_value = m2_encoder;		// Log encoder value
		m2_set_direction(RETRACT);
		delay(500);

		#ifndef SERIAL_MASTER_INIT_TALK_ONLY
		#ifdef DEBUG_PRINT
		Serial.print("11 EncoderValue:");
		Serial.print(m1_encoder);
		Serial.print(",");
		Serial.println(m2_encoder);
		#endif	
		#endif // !SERIAL_MASTER_INIT_TALK_ONLY

		//Handles if the motor is not moving in second attempt
		if (last_encoder_2_value - m2_encoder < 10) {
			#ifndef SERIAL_MASTER_INIT_TALK_ONLY
			Serial.println("Homing Fail, Motor 2 is not running in either direction");
			#endif // !SERIAL_MASTER_INIT_TALK_ONLY
			m1_stop();
			m2_stop();
			return;
		}
		//After motor is backed, we zero the encoder again and start extending again
		//Set speed to Extend Full Speed and wait for a while
		last_encoder_2_value = m2_encoder;
		m2_set_direction(EXTEND);
		delay(500);
	}

	// Wait until motor stops moving by detecting the encoder stop moving

	while (m1_encoder != last_encoder_1_value || m2_encoder != last_encoder_2_value) {
		last_encoder_1_value = m1_encoder;		// Log encoder value
		last_encoder_2_value = m2_encoder;		// Log encoder value
		delay(100);				// Delay and check if encode is still zero
	}
	m1_stop();
	m2_stop();
	axis_homed = true;
	#ifndef SERIAL_MASTER_INIT_TALK_ONLY
	Serial.print("Homing Completed. Steps:");
	Serial.print(m1_encoder);
	Serial.print(",");
	Serial.println(m2_encoder);
	#endif // !SERIAL_MASTER_INIT_TALK_ONLY

	m1_encoder = m1_default_home_position * step_per_mm; // Reset Encoder value to home position
	m2_encoder = m2_default_home_position * step_per_mm; // Reset Encoder value to home position
}

//This function is called when an interrupt is detected by the arduino
void encoder_trigger_1()
{
	if (m1_direction == EXTEND) {
		m1_encoder++;
	}
	else {
		m1_encoder--;
	}
}

//This function is called when an interrupt is detected by the arduino
void encoder_trigger_2()
{
	if (m2_direction == EXTEND) {
		m2_encoder++;
	}
	else {
		m2_encoder--;
	}
}
