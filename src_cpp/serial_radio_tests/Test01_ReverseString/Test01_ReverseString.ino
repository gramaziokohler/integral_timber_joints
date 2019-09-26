/*
 Name:		ReverseString.ino
 Created:	9/15/2019 3:26:43 PM
 Author:	leungp
*/

#include "reliable_messenger.h"


//Comment out this define to test simple Serial Messaging
#define _transport_via_radio

constexpr unsigned int  computerAddress = 97U;        //Address of the computer: char 'a' 
constexpr unsigned int  thisArduinoAddress = 98U;     //Address of this Arduino: char 'b'
constexpr unsigned int  thisArduinoFreq = 50U;     //50 = CFREQ_433

constexpr unsigned int reverseStringBufferSize = 128;  //Size of the string to be reversed;

SerialRadioTransport transport(Serial, reverseStringBufferSize);

ReliableMessenger messenger(transport);


// the setup function runs once when you press reset or power the board
void setup() {

    // Sending a welcome message by Hardware Serial object. (No automatically ACK)
    Serial.begin(115200);
    Serial.print("Serial Ready\n");

    // Sending a welcome message by transport layer. (No automatically ACK)
    transport.setAddress(thisArduinoAddress);
    transport.setEndOfMessageChar('\x0004');
    Message msp;
    msp.senderAddress = transport.getAddress();
    msp.receiverAddress = computerAddress;
    msp.body = "Transport Ready\n";
    transport.sendMessage(&msp);

    //Set Radio's Address to transport address
    msp.receiverAddress = 7;
    msp.senderAddress = 7;
    msp.body = "0 ";
    msp.body[1] = thisArduinoAddress;
    transport.sendMessage(&msp);

    //Set Radio's Frequency to CFREQ_433
    msp.receiverAddress = 7;
    msp.senderAddress = 7;
    msp.body = "1 ";
    msp.body[1] = thisArduinoFreq;
    transport.sendMessage(&msp);

    messenger.setSendTimeout(300);
}

// the loop function runs over and over again until power down or reset
char replyBuffer[reverseStringBufferSize]; // Buffer for reversing strings
void loop() {

    if (messenger.available()) {
        //Reveice the incoming message
        Message incomingMessage = *messenger.receiveMessage();

        auto length = incomingMessage.length();
        for (int i = 0; i < length; i++) {
            replyBuffer[i] = incomingMessage.body[length - 1 - i];
        }
        replyBuffer[length] = 0;     //Terminate char[]

        //Some pause before reply
        delayMicroseconds(10);

        //Construct a reply message
        Message replyMessage;
        replyMessage.receiverAddress = incomingMessage.senderAddress;
        replyMessage.senderAddress = incomingMessage.receiverAddress;
        //The reply message body is a char[] that needs to be constructed.
        replyMessage.body = replyBuffer;

        // Send reply message
        messenger.sendMessage(&replyMessage);
    }

}



// Test 1: Simple SerialTransport Class Behaviour:
//  Sending `0123\x0004` from computer to Arduino:
//  Received `ACKa\x0004` from Arduino - as ACK message
//  Received `3210\x0004` from Arduino - as reversed String content 

//  Test 2: Serial via SerialRadioTransport Behaviour:
//  Assume address of computer is `a`, address of arduino is `b`
//  Sending `ba0123\x0004` from computer to Arduino:
//  Received `abACKa\x0004` from Arduino - as ACK message
//  Received `ab3210\x0004` from Arduino - as reversed String content 
