/*
 Name:		_02_LoadCellInterface.ino
 Created:	7/28/2019 1:33:21 PM
 Author:	leungp
*/

// the setup function runs once when you press reset or power the board
#include <HX711.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

void setup() {
	Serial.begin(115200);

	Serial.println("HX711 Demo");

	Serial.println("Initializing the scale");

	// Initialize library with data output pin, clock input pin and gain factor.
	// Channel selection is made by passing the appropriate gain:
	// - With a gain factor of 64 or 128, channel A is selected
	// - With a gain factor of 32, channel B is selected
	// By omitting the gain factor parameter, the library
	// default "128" (Channel A) is used here.
	scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
	scale.set_gain(64);
	Serial.println("Before setting up the scale:");
	Serial.print("read: \t\t");
	Serial.println(scale.read());			// print a raw reading from the ADC

	Serial.print("read average: \t\t");
	Serial.println(scale.read_average(20));  	// print the average of 20 readings from the ADC

	Serial.print("get value: \t\t");
	Serial.println(scale.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight (not set yet)

	Serial.print("get units: \t\t");
	Serial.println(scale.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare weight (not set) divided
						  // by the SCALE parameter (not set yet)

	scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
	scale.tare();				        // reset the scale to 0

	Serial.println("After setting up the scale:");

	Serial.print("read: \t\t");
	Serial.println(scale.read());                 // print a raw reading from the ADC

	Serial.print("read average: \t\t");
	Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

	Serial.print("get value: \t\t");
	Serial.println(scale.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight, set with tare()

	Serial.print("get units: \t\t");
	Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
						  // by the SCALE parameter set with set_scale

	Serial.println("Readings:");
}

// the loop function runs over and over again until power down or reset
void loop() {


	Serial.print("one reading:\t");
	Serial.print(scale.get_units(), 1);
	Serial.print("\t| average:\t");
	Serial.println(scale.get_units(10), 1);

	// Testing to see how many time it can read in one second (~11 times)
	Serial.print("How many scale read in one Second: \t");
	long end_time = millis() + 1000;
	int count = 0;
	while (millis() < end_time) {
		scale.get_units();
		count++;
	}
	Serial.println(count);

	// Testing to see how long it take to read the values when the line is ready. (~530 microsecs)
	delay(1000);
	while (!scale.is_ready()) {}

	long timer = micros();
	scale.get_units();
	timer = micros() - timer;
	Serial.print("How many micro seconds it takes to read when ready: \t");
	Serial.println(timer);

	// Testing to see how long it take to be ready again. (91.6 millisecs)
	scale.get_units();
	timer = micros();
	while (!scale.is_ready()) {}
	timer = micros() - timer;
	Serial.print("How many micro seconds to wait to be ready again: \t");
	Serial.println(timer);


	scale.power_down();			        // put the ADC in sleep mode
	delay(5000);
	scale.power_up();
}
