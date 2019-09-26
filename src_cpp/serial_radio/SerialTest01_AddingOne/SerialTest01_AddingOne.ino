/*
 Name:		SerialTest01_AddingOne.ino
 Created:	7/12/2019 10:11:44 PM
 Author:	leungp
*/
#include <Arduino.h>

// Global Variable - Serial
const long USBSerialBaud = 115200;
const char SerialTernimation = 10; // 10 = LineFeed , 13 = CarrageReturn
const char BeginEscape = 40;
const char EndEscape = 41;

// Global Variable - Serial
char serialRxBuffer[100];			// Stores the incoming Serial Message - because it is received char by char
int serialRxLength;					// Stores the length of the incoming Serial Message - because it is received char by char
bool SerialEscaping = false;

//Check if USB message is available and waiting.
//Receive the message from USB Serial buffer and relay the message to USB Serial
void handleSerialMessage() {
	while (Serial.available()) {
		char c = Serial.read();  //gets one byte from serial buffer
		if (c == BeginEscape) {
			SerialEscaping = true;
			continue;
		}
		if (c == EndEscape) {
			SerialEscaping = false;
			continue;
		}
		if (SerialEscaping) continue;
		if (c == SerialTernimation) {
			serialRxBuffer[serialRxLength] = 0;
			ProcessMessage();
			serialRxLength = 0;
			serialRxBuffer[serialRxLength] = 0;
		}
		else {
			serialRxBuffer[serialRxLength] = c;
			serialRxLength++;
		}

	}
}

void ProcessMessage() {

	//Serial.print("[Message =");
	//Serial.print(serialRxBuffer);
	//Serial.print("]");
	//Serial.write(SerialTernimation);
	//delay(500);

	// If message is empty, return
	if (serialRxLength == 0) {
		//Serial.print("[Message is empty]");
		//Serial.write(SerialTernimation);
		return;
	}

	// Parse string to long integer, add 1 and return as String.
	long number = atol(serialRxBuffer);
	Serial.print(number + 1);
	Serial.write(SerialTernimation);
}

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(USBSerialBaud);
}

// the loop function runs over and over again until power down or reset
void loop() {
	handleSerialMessage();
}
