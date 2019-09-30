/*
 Name:		Test00_ACK.ino
 Created:	9/15/2019 3:26:43 PM
 Author:	leungp

 This sketch is uploaded to an UNO and is connected to the digital radio.
 It initialize radio address and frequency
 It keeps the SerialRadioTransport and ReliableMessenger running and ACK all incomming messages.
 

*/

#include "reliable_messenger.h"


//Comment out this define to test simple Serial Messaging
#define _transport_via_radio

constexpr unsigned int  thisArduinoAddress = 98U;     //Address of this Arduino: char 'b'
constexpr unsigned int  thisArduinoFreq = 50U;     //50 = CFREQ_433

constexpr unsigned int reverseStringBufferSize = 128;  //Size of the string to be reversed;

SerialRadioTransport transport(Serial, reverseStringBufferSize);

ReliableMessenger messenger(transport);


// the setup function runs once when you press reset or power the board
void setup() {

    // Configure Serial
    Serial.begin(115200);
    delay(1000);
    //Serial.print("Serial Ready\n");

    // Configure transport layer
    transport.setEndOfMessageChar('\x0004');
    transport.setRadioAddress(thisArduinoAddress); //This was done two times because one time doesnt work.
    transport.setRadioAddress(thisArduinoAddress);
    delay(500);
    transport.setRadioFrequency(thisArduinoFreq);
    transport.setRadioFrequency(thisArduinoFreq);
    delay(500);

    //Send remote a meessage
    Message msg = Message();
    msg.receiverAddress = 'a';
    msg.senderAddress = 'b';
    msg.body = "Remote Controller On";
    transport.sendMessage(& msg);

    // Configure messsenger
    messenger.setSendTimeout(300);
}

// the loop function runs over and over again until power down or reset
void loop() {

    if (messenger.available()) {
        //Reveice the incoming message
        Message incomingMessage = *messenger.receiveMessage();
        //Do something else maybe.
    }

}


//  Assume address of computer is `a`, address of arduino is `b`

// Test 1: Simple SerialTransport Class Behaviour:
//  Sending `ba23\x0004` from computer to Arduino:
//  Received `abACKg\x0004` from Arduino - as ACK message

//  Test 2: Serial via SerialRadioTransport Behaviour:
//  Sending `ba0123\x0004` from computer to Arduino:
//  Received `abACKa\x0004` from Arduino - as ACK message
