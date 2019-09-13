/*
 Name:		messenger_tester.ino
 Created:	9/12/2019 4:06:07 PM
 Author:	leungp
*/
#include <ReliableMessenger.h>


SerialTransport transport (Serial, 64);
ReliableMessenger messenger(transport);

void setup() {
    Serial.begin(115200);
    Serial.print("Serial ");
    Serial.print("Ready\n");

    // Sending Message by transport layer.
    Message msp;
    msp.body = "Transport ";
    transport.sendMessage(&msp);
    msp.body = "Ready\n";
    transport.sendMessage(&msp);

    //Sending message by messenger layer, retry as needed
    //messenger.setRetry(3);
    //messenger.setSendTimeout(300);
    msp.body = "Messenger ";
    messenger.sendMessage(&msp);
    msp.body = "Ready\n";
    messenger.sendMessage(&msp);

}

// the loop function runs over and over again until power down or reset
void loop() {

    if (messenger.available()) {
        Message newMessage = *messenger.receiveMessage();
        delay(1000);
        char replyBuffer[64];

        for (int i = 0; i < newMessage.length(); i++) {
            replyBuffer[i] = newMessage.body[newMessage.length() - 1 - i];
        }
        replyBuffer[newMessage.length()] = '\n';    //Append new Line Symbol for nicety
        replyBuffer[newMessage.length()+1] = 0;     //Terminate char[]

        Message replyMessage;
        replyMessage.body = replyBuffer;
        messenger.sendMessage(& replyMessage);
    }

}


// Test: Communicate with the messenger via Serial
// Send the controller "abcdefg\x0004"
// - `x0004` is the message termination character 
// The controller will reply with a "ACKL\x0004"
// - `ACK` is indicating the message is an ACK emssage
// - `L` is the check sum
// The controller will pretend to do something
// - Actually it is delay(1000)
// The controller will reply a dummy computed result: "gfedcba\n\0004"
// - The result is reversed string from the received message
// - `\n` attached to the end for easy display.
