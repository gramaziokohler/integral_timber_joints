/*
 Name:		reliable_messenger.cpp
 Created:	9/15/2019 12:49:36 PM
 Author:	leungp
 Editor:	http://www.visualmicro.com
*/


#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "reliable_messenger.h"

// cpp:class::Message
// -----------------------


int Message::length() const {
    return strlen(body);
}

// cpp:class::ReliableMessenger
// ------------------------------


ReliableMessenger::ReliableMessenger(Transport &transport) {
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
        incomingMessage = * _transport->receiveMessage();
        _newMessageFlag = true;
        sendACK(& incomingMessage);
        return true;
    }
    //Reachable if no message is available
    return false;
}

const Message * ReliableMessenger::receiveMessage() {
    if (!available()) return NULL;
    _newMessageFlag = false;
    return & incomingMessage;
}

int ReliableMessenger::sendMessage(Message const * message) {
    //Send the message
    _transport->sendMessage(message);
    //Compute the expected ACK Checksum String
    char expectedString[5];
   computeACKString(message->body,expectedString);

    //Wait for ACK and automatic retry
    int retryCount;
    for (retryCount = 0; retryCount < _retryMax; retryCount++) {
        unsigned long startTime = millis();
        while (millis() - startTime < _sendSendTimeoutMillis) {
            // Try to read incoming message
            if (_transport->available()) {
                // Compare incoming message to expected ACK value
                Message msg = *_transport->receiveMessage();
                //Serial.print("IncomingMsg:");

                if (strcmp(msg.body, expectedString) == 0 ) {
                    //Serial.println("ACK Heard Send is Success");
                    return retryCount;
                } else {
                    //Serial.println("Bad Compare");

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
    //Consume any unreceived messages from the transport layer.
    int count = 0;
    while (_transport->available()) {
        _transport->receiveMessage();
        count++;
    }
    //Consume any received but not read messages.
    _newMessageFlag = false; 
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

// Send ACK message back to the sender.
void ReliableMessenger::sendACK(Message * message) {
    Message ackMessage;
    ackMessage.senderAddress = _transport->getAddress();
    ackMessage.receiverAddress = message->senderAddress;
    //Construct the ACK message body
    char ackMessageBody[5];
    computeACKString(message->body, ackMessageBody);
    ackMessage.body = ackMessageBody;

    _transport->sendMessage(&ackMessage);
}

// Compute the ACK string according to a message content.
void ReliableMessenger::computeACKString(char * originalMessage, char * ackString) {
    strcpy(ackString, "ACK ");
    ackString[3] = stringChecksum(originalMessage);
}

// Compute checksum byte based on message content.
byte ReliableMessenger::stringChecksum(char *s) {
    byte c = 0;
    byte index = 0;
    while (*s != '\0') {
        c ^= ((*s++) + index); //Repeated Bitwise OR with (the char + index)
        index++;
    }
    return (c % 25) + 97; //Limits the returned byte to be ASCII a-z
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

}

SerialTransport::SerialTransport(Stream &stream, unsigned int bufferLength) {
    _receiveBuffer = new char[bufferLength]();
    _stream = &stream;

}

void SerialTransport::sendMessage(Message const * message) {

    _stream->write(message->body);
    _stream->write(_endOfMessageChar);
}

boolean SerialTransport::available() {
    //If Last Received Byte is a \0, then we should immediately return true without reading more data.
    if (_receiveIndex > 0) {
        if (_receiveBuffer[_receiveIndex - 1] == _endOfMessageChar) return true;
    }

    //Receive as much character as possible and return true if \0 is detected.
    if (_stream->available()) {
        char newChar = _stream->read();
        // End of message character - end char array with 0 and return true
        if (newChar == _endOfMessageChar) {
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

Message const * const SerialTransport::receiveMessage() {
    _receivedMessage.body = _receiveBuffer;
    _receivedMessage.receiverAddress = _address;
    _receivedMessage.senderAddress = 0U;
    _receiveIndex = 0; // reset Index
    return &_receivedMessage;
}

char SerialTransport::getEndOfMessageChar() {
    return _endOfMessageChar;
}

void SerialTransport::setEndOfMessageChar(char endOfMessageChar) {
    _endOfMessageChar = endOfMessageChar;
}


// cpp:class::SerialRadioTransport
// ----------------


SerialRadioTransport::SerialRadioTransport(Stream & stream, unsigned int bufferLength) :SerialTransport(stream, bufferLength) {
    //Calling only base class constructor
}

SerialRadioTransport::SerialRadioTransport(Stream & stream, unsigned int bufferLength, byte address) : SerialTransport(stream, bufferLength, address) {
    //Calling only base class constructor
}

void SerialRadioTransport::sendMessage(Message const * message) {
    _stream->write(message->receiverAddress);   //Byte 0
    _stream->write(message->senderAddress);     //Byte 1
    _stream->write(message->body);              //Byte 2 & onwards
    _stream->write(_endOfMessageChar);         //Byte final byte
}

boolean SerialRadioTransport::available() {
    //If Last Received Byte is a \0, then we should immediately return true without reading more data.
    if (_receiveIndex > 0) {
        if (_receiveBuffer[_receiveIndex - 1] == _endOfMessageChar) return true;
    }

    //Receive as much character as possible and return true if \0 is detected.
    while (_stream->available()) {
        char newChar = _stream->read();

        // If this is the first character, check if message is for me.
        if (_receiveIndex == 0) {
            if (newChar != _address) {
                messageForMe = false;   //Set messageForMe Flag . This stops subsquent storage.
            }
        }

        // End of message character - end char array with 0 and return true
        if (newChar == _endOfMessageChar) {
            if (messageForMe) {
                _receiveBuffer[_receiveIndex] = 0;  //Terminate char[]
                return true;
            } else { // Message is not for me but now it ends.
                _receiveIndex = 0;      //Reset Receive Index (This will consume the message)
                messageForMe = true;    //Reset messageForMe Flag
                continue;               //Skip the following storage in buffer
            }
        }
        // Normal character - proceed to store in buffer
        else {
            if (messageForMe) {
                _receiveBuffer[_receiveIndex] = newChar;
            }
        }
        _receiveIndex++;
    } // while (_stream->available()) 

    //Reachable when no \0 is detected and no characters are available from _stream.
    return false;
}

Message const * const SerialRadioTransport::receiveMessage() {
    _receivedMessage.receiverAddress = _receiveBuffer[0];
    _receivedMessage.senderAddress = _receiveBuffer[1];
    _receivedMessage.body = _receiveBuffer + 2;             // Message starts from byte 2
    _receiveIndex = 0;                                      // reset Index
    return &_receivedMessage;
}


// cpp:class::CC1101RadioTransport
// ----------------


//
//CC1101RadioTransport::CC1101RadioTransport(byte address) {
//    _address = address;
//
//}
//
//boolean CC1101RadioTransport::begin() {
//    return false;
//}
//
//void _CC1101RadioTransportMessageReceived() { CC1101RadioTransport::_messageReceived(); }
//
//void CC1101RadioTransport::sendMessage(Message const * message) {
//    //Detach Receive interrupt
//    detachInterrupt(_interruptPin);
//
//    //Construct new CCPACKET
//    CCPACKET packet;
//    packet.length = message->length();
//
//    //Copy message from serialRxBuffer to CCPACKET
//    strncpy((char *)packet.data, message->body, packet.length);
//
//    //Send message
//    _radio->sendData(packet);
//
//    //Print debug information to Serial
//
//    //Serial.print(F("(Sent packet,len="));
//    //Serial.print(serialRxLength);
//    //Serial.print(F(")"));
//    //Serial.write(SerialTernimation);
//
//    //Reattach Receive interrupt
//    attachInterrupt(_interruptPin, _CC1101RadioTransportMessageReceived, FALLING);
//
//}
//
//boolean CC1101RadioTransport::available() {
//    return boolean();
//}
//
//
