// This Serial Bridge code abstracts the [CC1101 Radio](http://www.ti.com/product/CC1101)
// that operates in [ISM Band](https://en.wikipedia.org/wiki/ISM_band) from another device 
// (such as a computer, another Arduino board or RPi board) that uses the radio functions. 
// It presents the radio functions to send and receive messages via a Serial Port interface.

//See Readme.md file for detailed information.

// CC1101 Radio Module should be wired to the following pins:
// CC1101 Pins => Arduino Pins
// 5V+ => 5V+
// GND => GND
// CSN (SS) => 10
// MOSI => 11
// MISO => 12
// SCK => 13
// GD0 => 2

// 1602 LCD Screen Expanded to I2C via PCF8574 IO Expander 
// Radio Pin => Arduino Pins
// Vcc => 5V+
// GND => GND
// SDA => A4 (SDA)
// SCL => A5 (SCL)

// Two test button (Pushbutton connected to Ground) can be attached to A0 and A1 for testing purpose

#include <Arduino.h>
#include <cc1101.h>
#include <ccpacket.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

#define CC1101Interrupt 0 // Pin 2 for a Uno or Nano
#define CC1101_GDO0 2

//Compile Option

//#define IncludeRadioQualityInSerialComment //If defined, received radio signal rssi and lqi will be included in the serial message as a bracket message after the real message.
//#define SerialComment // If defined, Radio will print debug message to Serial for debug purpose.

//Defines default Radio (network) settings - This is changable by user using Radio or Serial Commands
uint8_t this_radio_address = 89;
byte channelNumber = 19;
byte frequency = CFREQ_915; // CFREQ_868 CFREQ_915 CFREQ_433 CFREQ_918

 //Defines Radio Setting - This is not changable after compilation
constexpr byte syncWord[2] = { 01, 27 };

//Defines Serial Communication Settings
const long USBSerialBaud = 115200;
const char endOfMessageChar = 4; // 10 = LineFeed '\n' , 13 = CarrageReturn '\r' , 4 = CarrageReturn '\x004'
const char radiosettingChar = 7; // 10 = LineFeed '\n' , 13 = CarrageReturn '\r' , 4 = CarrageReturn '\x004'

// Global Variable - Radio
CC1101 radio;						// The object of the radio
bool packetWaiting;					// Stores the state set by radio inturrupt. If true, a packet is ready to be recieved from radio.
uint8_t radio_partnum, radio_version, radio_marcstate;	// Stores the register values of the radio fetched at startup.

// Global Variable - Serial
char serialRxBuffer[100];			// Stores the incoming Serial Message - because it is received char by char
int serialRxLength;					// Stores the length of the incoming Serial Message - because it is received char by char

// Global Variable - LCD
LiquidCrystal_PCF8574 lcd(0x27);	// LCD address to 0x27 for a 16 chars and 2 line display
bool lcdFound = false;				// Stores the state - if the LCD screen is found during starup.
bool radioFound = false;			// Stores the state - if the radio is found during starup.

//Function for Interrupt call back
void messageReceived() {
	packetWaiting = true;
}

// Radio helper function
// Get signal strength indicator in dBm.
// See: http://www.ti.com/lit/an/swra114d/swra114d.pdf
int rssi(char raw) {
	uint8_t rssi_dec;
	// TODO: This rssi_offset is dependent on baud and MHz; this is for 38.4kbps and 433 MHz.
	uint8_t rssi_offset = 74;
	rssi_dec = (uint8_t)raw;
	if (rssi_dec >= 128)
		return ((int)(rssi_dec - 256) / 2) - rssi_offset;
	else
		return (rssi_dec / 2) - rssi_offset;
}

// Radio helper function
// Get link quality indicator.
int lqi(char raw) {
	return 0x3F - raw;
}

// Routine to start LCD screen
void LCDStartUp() {
	// LCD Start Up
	Wire.begin();
	Wire.beginTransmission(0x27);
	int error;
	error = Wire.endTransmission();
	if (error == 0) {
		#if defined(SerialComment)
			Serial.print(F("(LCD found)"));
			Serial.write(endOfMessageChar);
		#endif
		lcdFound = true;
		lcd.begin(16, 2); // initialize the lcd
		lcd.setBacklight(255);
		lcd.home();
		lcd.clear();
		lcd.print(F("LCD OK"));
	}
	else {
		lcdFound = false;
		#if defined(SerialComment)
			Serial.print(F("(LCD not found - No Display)"));
			Serial.write(endOfMessageChar);
		#endif

	} // if
}

// Routine to start Radio
void RadioStartup() {
	radio.init();
	radio.setSyncWord(syncWord[0], syncWord[1]);
	radio.setDevAddress(this_radio_address);
	radio.setCarrierFreq(frequency);
	//radio.disableAddressCheck();
	radio.setTxPowerAmp(PA_LongDistance);
	radio.setChannel(channelNumber);

	radio_partnum = radio.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER);
	radio_version = radio.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER);
	radio_marcstate = radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;

	// Determine if radio is found. If everything is zero, the radio is probably dead.
	if (radio_partnum == 0 && radio_version == 0 && radio_marcstate == 0) {
		// Radio Not Found - report to Serial
		#if defined(SerialComment)
			Serial.print(F("(CC1101 radio not found.)"));
			Serial.write(endOfMessageChar);
		#endif
		// Radio Not Found - report to LCD
		if (lcdFound) {
			lcd.setCursor(0, 1);
			lcd.print(F("(Radio Bad)"));
		}
	}
	else {
		// Radio Found - Report to Serial
		#if defined(SerialComment)
			Serial.print(F("(CC1101 radio found.)"));
			Serial.write(endOfMessageChar);
			Serial.print(F("(CC1101_PARTNUM "));
			Serial.print(radio_partnum);
			Serial.print(F(")"));
			Serial.write(endOfMessageChar);
			Serial.print(F("(CC1101_VERSION "));
			Serial.print(radio_version);
			Serial.print(F(")"));
			Serial.write(endOfMessageChar);
			Serial.print(F("(CC1101_MARCSTATE "));
			Serial.print(radio_marcstate);
			Serial.print(F(")"));
			Serial.write(endOfMessageChar);
		#endif
		// Radio Found - Report to LCD
		if (lcdFound) {
			lcd.setCursor(0, 1);
			lcd.print(F("Radio OK"));
		}
	}

	attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
}

void setup() {
	Serial.begin(USBSerialBaud);

	pinMode(A0, INPUT_PULLUP);
	pinMode(A1, INPUT_PULLUP);

	LCDStartUp();
	RadioStartup();
}

//Check if package is available and waiting.
//Receive the message from radio buffer and relay the message to USB Serial
void receiveRadioMessage() {
	if (packetWaiting) {
		//Detatch receive Interrupt
		detachInterrupt(CC1101Interrupt);
		//Lower receive Interrupt Flag
		packetWaiting = false;

		//Create new CCPACKET for holding received information
		CCPACKET packet;
		if (radio.receiveData(&packet) > 0) {
			//Check if the packet has at least 3 bytes (2 bytes were header)
			if (packet.crc_ok && packet.length > 2) {
				// Check if message is a command - Serial Command have byte 0 being the settingChar
				if (packet.data[1] == radiosettingChar) {

					changeRadioSetting((char*) packet.data);
				}
				// If it is not a setting, pass on the message to Serial
				else {
					sendSerialMessage(& packet);
				}


			}
		}
		attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
	}
}

// Check if USB message is available and waiting.
// Receive the message from USB Serial buffer and place it in buffer ready for sending via radio
// Checks if the character is Serial Termination, if so, send the message.
void receiveSerialMessage() {
	while (Serial.available()) {
		char c = Serial.read();					//gets one byte from serial buffer
		
		//Check if c is the endOfMessageChar
		if (c == endOfMessageChar) {
			//serialRxBuffer[serialRxLength] = 0;	//Terminate the serial message // Not necessary because we keep track of serialRxLength

			//Checkes if the message has at least 3 bytes (2 bytes were header)
			if (serialRxLength > 2) {
				// Check if message is a command - Serial Command have byte 0 being the settingChar
				if (serialRxBuffer[0] == radiosettingChar) {
					changeRadioSetting(serialRxBuffer);
				}
				// If it is not a setting, pass on the message
				else {
					//Send the radio message only if it has at least the 2 byte header.
					sendRadioMessage(); //Send message from serialRxBuffer[].
				}
			}
			//delayMicroseconds(10);
			//Reset serialRxLength Counter after message is sent.
			serialRxLength = 0;					
		}
		//Store the character in the message body
		else {
			serialRxBuffer[serialRxLength] = c; //Place incoming Serial character in Buffer
			serialRxLength++;					//Increment the serialRxLength Counter.
		}

	}
}

// Relays message from incoming Serial to outgoing Radio 
void sendRadioMessage() {
	//Detach Receive interrupt
	detachInterrupt(CC1101Interrupt);

	//Construct new CCPACKET
	CCPACKET packet;
	packet.length = serialRxLength;

	//Copy message from serialRxBuffer to CCPACKET
	strncpy((char *)packet.data, serialRxBuffer, packet.length);

	//Send message
	radio.sendData(packet);

	//Print debug information to Serial
	#if defined(SerialComment)
		Serial.print(F("(Sent packet,len="));
		Serial.print(serialRxLength);
		Serial.print(F(")"));
		Serial.write(endOfMessageChar);
	#endif

	//Print debug information to LCD
	if (lcdFound) {
		lcd.clear();
		lcd.setCursor(0, 1);
		lcd.print(">");
		for (int i = 0; i < packet.length; i++) {
			lcd.write(packet.data[i]);
		}
	}
	//Reattach Receive interrupt
	attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
}

// Relays message from incoming Radio to outgoing Serial 
void sendSerialMessage(CCPACKET * _packet) {
	CCPACKET packet = * _packet;

	//Print the message to Serial (Note the 2 address bytes are inside the packet.data[])
	for (int i = 0; i < packet.length; i++) {
		Serial.write(packet.data[i]);
	}

	//Because the LF Termination byte is not transmitted via radio, it is added back here.
	Serial.write(endOfMessageChar);
	
	//Print radio quality to Serial
	#if defined(IncludeRadioQualityInSerialComment)
		//Print the lqi and rssi)
	Serial.print(F("("));
	//Serial.print(lqi(packet.lqi));
	Serial.print(packet.lqi);
	Serial.print(F(","));
	//Serial.print(rssi(packet.rssi));
	Serial.print(packet.rssi);
	Serial.print(F(")"));
	Serial.write(endOfMessageChar);
	#endif

	//Print message and radio quality to LCD
	if (lcdFound) {
		//lcd.clear();
		//Top Row
		lcd.setCursor(0, 0);
		lcd.print("rs:");
		lcd.print(rssi(packet.rssi));
		lcd.print("      ");
		lcd.setCursor(9, 0);
		lcd.print("lq:");
		lcd.print(lqi(packet.lqi));
		lcd.print("      ");
		// Bottom row
		lcd.setCursor(0, 1);
		lcd.print("<");
		for (int i = 0; i < 16; i++) {
			if (i < packet.length) lcd.write(packet.data[i]);
			else lcd.write(' ');
		}
	}
}

void changeRadioSetting(char const *  const buffer) {

	//Determine which setting to check

	//Setting '0' - Radio Address
	if (buffer[2] == '0') {
		uint8_t address = (uint8_t)buffer[3];
		radio.setDevAddress(address);
		//Debug Printout
		{
			//Debug to LCD
			if (lcdFound) {
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Set: Addr:");
				lcd.setCursor(0, 1);
				lcd.print("(uint)");
				lcd.print(address);
			}
			// Debug to Serial
			#if defined(SerialComment)
			Serial.print(F("(Radio Address Changed to: "));
			Serial.print(address);
			Serial.print(F(")"));
			Serial.print(endOfMessageChar);
			#endif
		}
	}
		//Setting '1' - Radio Frequency
	if (buffer[2] == '1') {
		char freq = (char)buffer[3];
		if (freq >= '0' && freq <= '3') {
			radio.setCarrierFreq(freq - 48); //-48 is to convert Char '0' to numerical 0
		} else return;
		//Debug Printout
		{
			//Debug to LCD
			if (lcdFound) {
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Set: Freq:");
				lcd.setCursor(0, 1);
				if (freq == '0') lcd.print("0 = 868Hz");
				if (freq == '1') lcd.print("1 = 915Hz");
				if (freq == '2') lcd.print("2 = 433Hz");
				if (freq == '3') lcd.print("3 = 918Hz");
			}
			// Debug to Serial
			#if defined(SerialComment)
			Serial.print(F("(Radio Frequency Changed to: "));
			if (freq == '0') Serial.print("0 = 868Hz");
			if (freq == '1') Serial.print("1 = 915Hz");
			if (freq == '2') Serial.print("2 = 433Hz");
			if (freq == '3') Serial.print("3 = 918Hz");
			Serial.print(F(")"));
			Serial.print(endOfMessageChar);
			#endif
		}
	}

	//Setting '2' - Radio Channel
	if (buffer[2] == '2') {
		uint8_t channel = (uint8_t)buffer[3];
		radio.setChannel(channel);
		//Debug Printout
		{
			//Debug to LCD
			if (lcdFound) {
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Set: Channel:");
				lcd.setCursor(0, 1);
				lcd.print("(uint)");
				lcd.print(channel);
			}
			// Debug to Serial
			#if defined(SerialComment)
			Serial.print(F("(Radio Channel Changed to: "));
			Serial.print(channel);
			Serial.print(F(")"));
			Serial.print(endOfMessageChar);
			#endif
		}
	}

	
}

long increasingNumber = 0;

// Main loop services both Serial and Radio recption
void loop() {
	receiveRadioMessage();
	receiveSerialMessage();
}
