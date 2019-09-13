/*
 Name:		messenger_tester.ino
 Created:	9/12/2019 4:06:07 PM
 Author:	leungp
*/
#include <ReliableMessenger.h>

// the setup function runs once when you press reset or power the board
ReliableMessenger myMessenger(Serial, onReceive);

void onReceive(String message) {
    // Special case where I expect only one message to be received.
    // Clear any unread messages before 
    myMessenger.discardUnreadMessages();
    
    Serial.println (message);
}




void setup() {
    Serial.begin(115200);
    myMessenger.setPin(13);
    myMessenger.dot();
    Serial.setTimeout(0);
}

// the loop function runs over and over again until power down or reset

char buffer[64];

void loop() {
    myMessenger.listen();

}
