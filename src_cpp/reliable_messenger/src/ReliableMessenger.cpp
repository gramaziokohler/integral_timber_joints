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


ReliableMessenger::ReliableMessenger(Transport &transport){
    _transport = &transport;
    _newMessageFlag = false;
}

// Checks if there is message waiting in the transport layer
// Automatically sends ACK when this is confirmed.
boolean ReliableMessenger::available() {
    // The flag is to ensure multiple calls to available() will not corrupt message
    if (_newMessageFlag) return true;
    // Receive the message and immediately send ACK.
    if (_transport->available()) {
        incomingMessage = _transport->receiveMessage();
        _newMessageFlag = true;
        sendACK(incomingMessage);
        return true;
    }
    //Reachable if no message is aailable
    return false;
}

Message * ReliableMessenger::receiveMessage() {
    if (!available()) return NULL;
    _newMessageFlag = false;
    return incomingMessage;
}

int ReliableMessenger::sendMessage(Message * message) {
    _transport->sendMessage(message);
    //Compute the expected ACK Checksum
    char * expectedString = computeACKString(message);
    
    //Wait for ACK and automatic retry
    int retryCount;
    unsigned long startTime = millis();
    for (retryCount = 0; retryCount < _retryMax; retryCount++) {
        while (millis() - startTime > _sendSendTimeoutMillis) {
            // Try to read incoming message
            if (_transport->available()) {
                // Compare incoming message to expected ACK value
                Message msg = * _transport->receiveMessage();
                if (strcmp(msg.body, expectedString)) {
                    return retryCount;
                }
            }
            // Confirm if the returned Message is correct
            // Return if the ACK message is correct
        }
        // Resend message
        if (retryCount < _retryMax - 1) _transport->sendMessage(message);
    }
    return -1;
}


// Discard unread messages from the Transport Layer
// returns: number of discard messages
unsigned int ReliableMessenger::discardUnreadMessages() {
    int count = 0;
    while (_transport->available()) {
        _transport->receiveMessage();
        count++;
    }
    return count;
}

// Sets the number of retry when sending message.
void ReliableMessenger::setRetry(int retryMax) {
    _retryMax = retryMax;
}

// Sets the duration of ACK timeout for sending message.
// This timeout applies to each retry.
void ReliableMessenger::setSendTimeout(unsigned long millisec) {
    _sendSendTimeoutMillis = millisec;
}

void ReliableMessenger::sendACK(Message * message) {
    Message ackMessage;
    ackMessage.senderAddress = _transport->getAddress();
    ackMessage.receiverAddress = message->senderAddress;
    ackMessage.body = computeACKString(message);

    Serial.println(ackMessage.body);
    _transport->sendMessage(&ackMessage);
}

// TODO
// Message is corrupted on return. Probably because ackString went out of scope.
char * ReliableMessenger::computeACKString(Message * message) {
    char ackString [] =  "ACK ";
    ackString[3] = stringChecksum(message->body);
    return ackString;
}

byte ReliableMessenger::stringChecksum(char *s) {
    byte c = 0;
    byte index = 0;
    while (*s != '\0') {
        c ^= ((*s++) + index); //Repeated Bitwise OR with (the char + index)
        index++;
    }
    return (c % 25)+ 65;
}


// cpp:class::Transport 
// ----------------

// Sets the address 
void Transport::setAddress(byte address) {
    _address = address;
}

byte Transport::getAddress() {
    return _address;
}

// cpp:class::SerialTransport
// ----------------

SerialTransport::SerialTransport(Stream &stream, unsigned int bufferLength, byte address) {
    _receiveBuffer = new char[bufferLength]();
    _stream = &stream;
    setAddress(address); //Default address
    _receiveIndex = 0;
}

SerialTransport::SerialTransport(Stream &stream, unsigned int bufferLength) {
    _receiveBuffer = new char[bufferLength]();
    _stream = &stream;
    setAddress(1); //Default address
    _receiveIndex = 0;
}

/// Send message to the connected serial device 
void SerialTransport::sendMessage(Message * message) {

    _stream->write(message->body);
    _stream->write(serialEndOfMessage);
}

boolean SerialTransport::available() {
    //If Last Received Byte is a \0, then we should immediately return true without reading more data.
    if (_receiveIndex > 0) {
        if (_receiveBuffer[_receiveIndex - 1] == serialEndOfMessage) return true;
    }

    //Receive as much character as possible and return true if \0 is detected.
    if (_stream->available()) {
        char newChar =  _stream->read();
        // End of message character - end char array with 0 and return true
        if (newChar == serialEndOfMessage) {
            _receiveBuffer[_receiveIndex] = 0;
            return true;
        }
        // Normal character - proceed to store in buffer
        else {
            _receiveBuffer[_receiveIndex] = newChar;
            _receiveIndex++;
        }
    }

    //Reachable when no \0 is detected and no characters are available from _stream.
    return false;
}

Message * SerialTransport::receiveMessage() {
    _receivedMessage.body = _receiveBuffer;
    _receivedMessage.receiverAddress = _address;
    _receivedMessage.senderAddress = 0U;
    _receiveIndex = 0; // reset Index
    return & _receivedMessage;
}

int Message::length() {
    return strlen(body);
}
