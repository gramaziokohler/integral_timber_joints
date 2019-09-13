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


ReliableMessenger::ReliableMessenger(Stream & stream, void(*receiveCallback)(char*)) {
    _stream = &stream;
    _receiveCallback = receiveCallback;
    _receivedLength = 0;
}

void ReliableMessenger::listen() {
    if (_stream -> available()) {
        //_stream->println("Stream available");
        _lastReceivedByte = _stream->read();
        if (_lastReceivedByte == 'p') {
            //Message Termination Encountered : Raise Callback
            _receiveBuffer[_receivedLength] = '\0'; //Terminate the char*
            _receiveCallback(_receiveBuffer);
            _receivedLength=0;
            // Jumps out of the listen loop to ensure only one callback is fired per listen() call.
            return;
        } else {
            //Save Byte to buffer and increment index counter
            _receiveBuffer[_receivedLength] = _lastReceivedByte;
            _receivedLength++; 
        }
    }
}

unsigned int ReliableMessenger::discardUnreadMessages() {
    unsigned int numberOfBytesAvailable = _stream->available();
    for (int i = numberOfBytesAvailable; i > 0 ; i--) {
        _stream->read();
    }
    return numberOfBytesAvailable;
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

void Transport::setMessageEndSymbol(char symbol) {
    _messageEndSymbol = symbol;
}

void Transport::setAddress(byte selfAddress) {
    _address = selfAddress;
}

// Serial Transport

SerialTransport::SerialTransport(Stream &stream, unsigned int bufferLength) {
    _receiveBuffer = new char[bufferLength]();
    _stream = &stream;
    setAddress(1); //Default address
    _receiveIndex = 0;

}

void SerialTransport::sendMessage(Message message) {
    _stream->write(message.body);
    _stream->write((uint8_t)0U);
}

boolean SerialTransport::available() {
    //If Last Received Byte is a \0, then we should immediately return true without reading more data.
    if (_receiveIndex > 0) {
        if (_receiveBuffer[_receiveIndex - 1] == '\0') return true;
    }

    //Receive as much character as possible and return true if \0 is detected.
    if (_stream->available()) {
        char newChar =  _stream->read();
        //_stream->print("Index:");
        //_stream->print(_receiveIndex);
        //_stream->print("=");
        //_stream->println(newChar);
        _receiveBuffer[_receiveIndex] = newChar;
        _receiveIndex++;
        if (newChar == '\0') return true;
    }

    //Reachable when no \0 is detected and no characters are available from _stream.
    return false;
}


Message SerialTransport::receiveMessage() {
    _receivedMessage.body = _receiveBuffer;
    _receivedMessage.receiverAddress = _address;
    _receivedMessage.senderAddress = 0U;
    _receiveIndex = 0; // reset Index
    return _receivedMessage;
}
