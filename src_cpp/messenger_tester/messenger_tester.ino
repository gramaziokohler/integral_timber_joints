/*
 Name:		messenger_tester.ino
 Created:	9/12/2019 4:06:07 PM
 Author:	leungp
*/
#include <ReliableMessenger.h>

void onReceive(String message) {
    // Special case where I expect only one message to be received.
    // Clear any unread messages before 
    //myMessenger.discardUnreadMessages();
    
    Serial.println (message);
}

// the setup function runs once when you press reset or power the board
ReliableMessenger myMessenger(Serial,onReceive);


void setup() {
    Serial.begin(115200);
    myMessenger.setPin(13);
    myMessenger.dot();
}

// the loop function runs over and over again until power down or reset
void loop() {
    myMessenger.listen();
}
