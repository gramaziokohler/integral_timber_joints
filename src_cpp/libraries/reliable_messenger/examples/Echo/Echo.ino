/*
 Name:		Echo.ino
 Created:	9/15/2019 12:50:22 PM
 Author:	leungp
*/


// the setup function runs once when you press reset or power the board

#include <reliable_messenger.h>

SerialTransport transport(Serial, 64);
ReliableMessenger messenger(transport);

void setup() {

    // Sending a welcome message by Hardware Serial object. (No automatically ACK)
    Serial.begin(115200);
    Serial.print("Serial Ready\n");

    // Sending a welcome message by transport layer. (No automatically ACK)
    Message msp;
    msp.body = "Transport Ready\n";
    transport.sendMessage(&msp);

    messenger.setSendTimeout(1000);

}

// the loop function runs over and over again until power down or reset
void loop() {

    //if (transport.available()) {
    //    //Reveice the incoming message
    //    Message incomingMessage = *transport.receiveMessage();

    //    //Construct a reply message
    //    Message replyMessage;
    //    replyMessage.receiverAddress = incomingMessage.senderAddress;
    //    replyMessage.senderAddress = incomingMessage.receiverAddress;
    //    replyMessage.body = incomingMessage.body;

    //    // Send reply message
    //    transport.sendMessage(&replyMessage);
    //}

    
    if (messenger.available()) {
        //Reveice the incoming message
        Message incomingMessage = *messenger.receiveMessage();

        //Construct a reply message
        Message replyMessage;
        replyMessage.receiverAddress = incomingMessage.senderAddress;
        replyMessage.senderAddress = incomingMessage.receiverAddress;
        //The reply message body is a char[] that needs to be constructed.
        char replyMessageBody[64];
        strcpy(replyMessageBody, incomingMessage.body);
        replyMessage.body = replyMessageBody;

        // Send reply message
        messenger.sendMessage(&replyMessage);
    }
}
