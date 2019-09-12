/*
 Name:		reliable_messenger.cpp
 Created:	9/11/2019 7:30:59 PM
 Author:	leungp
 Editor:	http://www.visualmicro.com
*/

#if defined(ARDUINO) && ARDUINO >= 100
    #include "arduino.h"
#else
    #include "WProgram.h"
#endif

#include "ReliableMessenger.h"


ReliableMessenger::ReliableMessenger(Stream & stream, void(*receiveCallback)(String)) {
    _stream = &stream;
    _receiveCallback = receiveCallback;
}

void ReliableMessenger::listen() {
    if (_stream -> available()) {
        _lastReceivedByte = _stream->read();
        if (_lastReceivedByte == 'p') {
            //Message Termination Encountered : Raise Callback
            _receiveCallback(_receiveBuffer);
            _receiveBuffer = "";
            // Jumps out of the listen loop to ensure only one callback is fired per listen() call.
            return;
        } else {
            //Save Byte to buffer
            
            _receiveBuffer += (char)_lastReceivedByte;
        }
    }
}

void ReliableMessenger::setPin(int pin) {
    pinMode(pin, OUTPUT);
    _pin = pin;
}

void ReliableMessenger::dot() {
    digitalWrite(_pin, HIGH);
    delay(250);
    digitalWrite(_pin, LOW);
    delay(250);
}

void ReliableMessenger::dash() {
    digitalWrite(_pin, HIGH);
    delay(1000);
    digitalWrite(_pin, LOW);
    delay(250);
}
